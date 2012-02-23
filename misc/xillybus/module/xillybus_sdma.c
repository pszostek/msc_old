/*
 * xillybus_sdma.c
 *
 * This file contains SDMA interface for the Xillybus driver.
 *
 * Copyright 2012 Xillybus Ltd.
 *
 * Based upon the general-purpose SDMA driver, drivers/dma/imx-sdma.c
 * Copyright 2010 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 *
 * Based on code from Freescale:
 *
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>

#include <asm/irq.h>
#include <mach/sdma.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/iomux-mx51.h>
#include <linux/gpio.h>

#include "xillybus_sdma.h"

#define THIS "xillybus_sdma: "

/* SDMA registers */
#define SDMA_H_C0PTR		0x000
#define SDMA_H_INTR		0x004
#define SDMA_H_STATSTOP		0x008
#define SDMA_H_START		0x00c
#define SDMA_H_EVTOVR		0x010
#define SDMA_H_DSPOVR		0x014
#define SDMA_H_HOSTOVR		0x018
#define SDMA_H_EVTPEND		0x01c
#define SDMA_H_DSPENBL		0x020
#define SDMA_H_RESET		0x024
#define SDMA_H_EVTERR		0x028
#define SDMA_H_INTRMSK		0x02c
#define SDMA_H_PSW		0x030
#define SDMA_H_EVTERRDBG	0x034
#define SDMA_H_CONFIG		0x038
#define SDMA_ONCE_ENB		0x040
#define SDMA_ONCE_DATA		0x044
#define SDMA_ONCE_INSTR		0x048
#define SDMA_ONCE_STAT		0x04c
#define SDMA_ONCE_CMD		0x050
#define SDMA_EVT_MIRROR		0x054
#define SDMA_ILLINSTADDR	0x058
#define SDMA_CHN0ADDR		0x05c
#define SDMA_ONCE_RTB		0x060
#define SDMA_XTRIG_CONF1	0x070
#define SDMA_XTRIG_CONF2	0x074
#define SDMA_CHNENBL0_V2	0x200
#define SDMA_CHNENBL0_V1	0x080
#define SDMA_CHNPRI_0		0x100

/*
 * Buffer descriptor status values.
 */
#define BD_DONE  0x01
#define BD_WRAP  0x02
#define BD_CONT  0x04
#define BD_INTR  0x08
#define BD_RROR  0x10
#define BD_LAST  0x20
#define BD_EXTD  0x80

/*
 * Data Node descriptor status values.
 */
#define DND_END_OF_FRAME  0x80
#define DND_END_OF_XFER   0x40
#define DND_DONE          0x20
#define DND_UNUSED        0x01

/*
 * IPCV2 descriptor status values.
 */
#define BD_IPCV2_END_OF_FRAME  0x40

#define IPCV2_MAX_NODES        50
/*
 * Error bit set in the CCB status field by the SDMA,
 * in setbd routine, in case of a transfer error
 */
#define DATA_ERROR  0x10000000

/*
 * Buffer descriptor commands.
 */
#define C0_ADDR             0x01
#define C0_LOAD             0x02
#define C0_DUMP             0x03
#define C0_SETCTX           0x07
#define C0_GETCTX           0x03
#define C0_SETDM            0x01
#define C0_SETPM            0x04
#define C0_GETDM            0x02
#define C0_GETPM            0x08
/*
 * Change endianness indicator in the BD command field
 */
#define CHANGE_ENDIANNESS   0x80

/*
 * EIM defines
 */
#define MX51_GPIO2_ISR (MX51_IO_ADDRESS(MX51_GPIO2_BASE_ADDR) + 0x18)
#define MX51_GPIO2_IMR (MX51_IO_ADDRESS(MX51_GPIO2_BASE_ADDR) + 0x14)
#define MX51_GPIO2_ICR1 (MX51_IO_ADDRESS(MX51_GPIO2_BASE_ADDR) + 0x0c)
#define MX51_GPIO2_ICR2 (MX51_IO_ADDRESS(MX51_GPIO2_BASE_ADDR) + 0x10)
#define MXC_CCM_CBCDR 0x14

/*
 * IO mux defines. To be patched into
 * arch/arm/plat-mxc/include/mach/iomux-mx51.h
 */

#define _MX51_PAD_GPIO1_5__SDMA_EXT_EVENT	IOMUX_PAD(0x808, 0x3dc, 1, 0x0000, 0, 0)
#define MX51_PAD_GPIO1_5__SDMA_EXT_EVENT	(_MX51_PAD_GPIO1_5__SDMA_EXT_EVENT | MUX_PAD_CTRL(NO_PAD_CTRL))

void __iomem *cs2_base = NULL;
static const int event_gpio_num = 5 + 0 * 32; /* gpio1/5 (event 15) */

/*
 * Three yucky global variables, containing metadata for the Xillybus client
 * for which this module is intended. Not extendible, but there's nothing to
 * extend on this platform.
 */

void *xillybus_userdata = NULL;
irq_handler_t xillybus_handler = NULL;
struct device *xillybus_dev = NULL;
struct sdma_channel *xillybus_sdmac = NULL;

/*
 * Mode/Count of data node descriptors - IPCv2
 */
struct sdma_mode_count {
	u32 count   : 16; /* size of the buffer pointed by this BD */
	u32 status  :  8; /* E,R,I,C,W,D status bits stored here */
	u32 command :  8; /* command mostlky used for channel 0 */
};

/*
 * Buffer descriptor
 */
struct sdma_buffer_descriptor {
	struct sdma_mode_count  mode;
	u32 buffer_addr;	/* address of the buffer described */
	u32 ext_buffer_addr;	/* extended buffer address */
} __attribute__ ((packed));

/**
 * struct sdma_channel_control - Channel control Block
 *
 * @current_bd_ptr	current buffer descriptor processed
 * @base_bd_ptr		first element of buffer descriptor array
 * @unused		padding. The SDMA engine expects an array of 128 byte
 *			control blocks
 */
struct sdma_channel_control {
	u32 current_bd_ptr;
	u32 base_bd_ptr;
	u32 unused[2];
} __attribute__ ((packed));

/**
 * struct sdma_state_registers - SDMA context for a channel
 *
 * @pc:		program counter
 * @t:		test bit: status of arithmetic & test instruction
 * @rpc:	return program counter
 * @sf:		source fault while loading data
 * @spc:	loop start program counter
 * @df:		destination fault while storing data
 * @epc:	loop end program counter
 * @lm:		loop mode
 */
struct sdma_state_registers {
	u32 pc     :14;
	u32 unused1: 1;
	u32 t      : 1;
	u32 rpc    :14;
	u32 unused0: 1;
	u32 sf     : 1;
	u32 spc    :14;
	u32 unused2: 1;
	u32 df     : 1;
	u32 epc    :14;
	u32 lm     : 2;
} __attribute__ ((packed));

/**
 * struct sdma_context_data - sdma context specific to a channel
 *
 * @channel_state:	channel state bits
 * @gReg:		general registers
 * @mda:		burst dma destination address register
 * @msa:		burst dma source address register
 * @ms:			burst dma status register
 * @md:			burst dma data register
 * @pda:		peripheral dma destination address register
 * @psa:		peripheral dma source address register
 * @ps:			peripheral dma status register
 * @pd:			peripheral dma data register
 * @ca:			CRC polynomial register
 * @cs:			CRC accumulator register
 * @dda:		dedicated core destination address register
 * @dsa:		dedicated core source address register
 * @ds:			dedicated core status register
 * @dd:			dedicated core data register
 */
struct sdma_context_data {
	struct sdma_state_registers  channel_state;
	u32  gReg[8];
	u32  mda;
	u32  msa;
	u32  ms;
	u32  md;
	u32  pda;
	u32  psa;
	u32  ps;
	u32  pd;
	u32  ca;
	u32  cs;
	u32  dda;
	u32  dsa;
	u32  ds;
	u32  dd;
	u32  scratch0;
	u32  scratch1;
	u32  scratch2;
	u32  scratch3;
	u32  scratch4;
	u32  scratch5;
	u32  scratch6;
	u32  scratch7;
} __attribute__ ((packed));

#define NUM_BD (int)(PAGE_SIZE / sizeof(struct sdma_buffer_descriptor))

struct sdma_engine;

/**
 * struct sdma_channel - housekeeping for a SDMA channel
 *
 * @sdma		pointer to the SDMA engine for this channel
 * @channel		the channel number, matches dmaengine chan_id + 1
 * @direction		transfer type. Needed for setting SDMA script
 * @peripheral_type	Peripheral type. Needed for setting SDMA script
 * @event_id0		aka dma request line
 * @event_id1		for channels that use 2 events
 * @word_size		peripheral access size
 * @buf_tail		ID of the buffer that was processed
 * @done		channel completion
 * @num_bd		max NUM_BD. number of descriptors currently handling
 */
struct sdma_channel {
	struct sdma_engine		*sdma;
	unsigned int			channel;
	enum dma_data_direction		direction;
	enum sdma_peripheral_type	peripheral_type;
	unsigned int			event_id0;
	unsigned int			event_id1;
	enum dma_slave_buswidth		word_size;
	unsigned int			buf_tail;
	struct completion		done;
	unsigned int			num_bd;
	struct sdma_buffer_descriptor	*bd;
	dma_addr_t			bd_phys;
	unsigned int			pc_from_device, pc_to_device;
	unsigned long			flags;
	dma_addr_t			per_address;
	u32				event_mask0, event_mask1;
	u32				watermark_level;
	u32				shp_addr, per_addr;
	struct dma_chan			chan;
	spinlock_t			lock;
	struct dma_async_tx_descriptor	desc;
	dma_cookie_t			last_completed;
	enum dma_status			status;
};

#define IMX_DMA_SG_LOOP		(1 << 0)

#define MAX_DMA_CHANNELS 32
#define MXC_SDMA_DEFAULT_PRIORITY 1
#define MXC_SDMA_MIN_PRIORITY 1
#define MXC_SDMA_MAX_PRIORITY 7

#define SDMA_FIRMWARE_MAGIC 0x414d4453

/**
 * struct sdma_firmware_header - Layout of the firmware image
 *
 * @magic		"SDMA"
 * @version_major	increased whenever layout of struct sdma_script_start_addrs
 *			changes.
 * @version_minor	firmware minor version (for binary compatible changes)
 * @script_addrs_start	offset of struct sdma_script_start_addrs in this image
 * @num_script_addrs	Number of script addresses in this image
 * @ram_code_start	offset of SDMA ram image in this firmware image
 * @ram_code_size	size of SDMA ram image
 * @script_addrs	Stores the start address of the SDMA scripts
 *			(in SDMA memory space)
 */
struct sdma_firmware_header {
	u32	magic;
	u32	version_major;
	u32	version_minor;
	u32	script_addrs_start;
	u32	num_script_addrs;
	u32	ram_code_start;
	u32	ram_code_size;
};

struct sdma_engine {
	struct device			*dev;
	struct device_dma_parameters	dma_parms;
	struct sdma_channel		channel[MAX_DMA_CHANNELS];
	struct sdma_channel_control	*channel_control;
	void __iomem			*regs;
	unsigned int			version;
	unsigned int			num_events;
	struct sdma_context_data	*context;
	dma_addr_t			context_phys;
	struct dma_device		dma_device;
	struct clk			*clk;
	struct sdma_script_start_addrs	*script_addrs;
};

#define SDMA_H_CONFIG_DSPDMA	(1 << 12) /* indicates if the DSPDMA is used */
#define SDMA_H_CONFIG_RTD_PINS	(1 << 11) /* indicates if Real-Time Debug pins are enabled */
#define SDMA_H_CONFIG_ACR	(1 << 4)  /* indicates if AHB freq /core freq = 2 or 1 */
#define SDMA_H_CONFIG_CSM	(3)       /* indicates which context switch mode is selected*/

static u32 bitfield(int shift, int bits, int val) {
	return ((val & ( ( 1 << bits ) - 1 ) ) << shift);
}

static void writereg(int offset, u32 val) {
	__raw_writel(val, MX51_IO_ADDRESS(MX51_WEIM_BASE_ADDR) + offset);
}

static int eim_init(void)
{
	int ret = 0;

	const int CSREC = 3;
	const int PSZ = 0;
	const int AUS = 1;
	const int BCS = 0;
	const int BCD = 0;
	const int BL = 3;
	const int WFL = 1;
	const int RFL = 1;
	const int WC = 0;
	const int ADH = 0;
	const int RWSC = 3;
	const int WWSC = 1;
	const int ADVA = 0; /* RADVA and WADVA */
	const int ADVN = 0; /* RADVN and WADVN */
	const int OEA = 0;
	const int CSA = 0; /* RCSA and WCSA */
	const int RL = 0;
	const int BEA = 0;
	const int BE = 1;
	const int WEA = 0;
	const int INTPOL = 1; /* Interrupt polarity */
	const int INTEN = 0; /* Interrupt enable */
	const int GBCD = 0; /* Burst clock divisor */
	const int BCM = 1; /* Burst clock mode (set c ontinuous here) */
  
	u32 GCR1, GCR2, RCR1, RCR2, WCR1, WEIMCR;
	u32 temp_clk;
	const int emi_slow_podf = 7;
  
	iomux_v3_cfg_t iomux_cs2 = MX51_PAD_EIM_CS2__EIM_CS2;
	iomux_v3_cfg_t iomux_dtack = MX51_PAD_EIM_DTACK__GPIO2_31;
	iomux_v3_cfg_t iomux_event = MX51_PAD_GPIO1_5__SDMA_EXT_EVENT;

	mxc_iomux_v3_setup_pad(iomux_cs2);
	mxc_iomux_v3_setup_pad(iomux_dtack);
	mxc_iomux_v3_setup_pad(iomux_event);

	/*
	 * Change the clock divider for EMI bus from divide-by-7 to divide-by-8, so
	 * that the bus frequency is below 90 MHz, in accordance with footnote 4 in
	 * section 4.6.7.3 in the datasheet. This is necessary to prevent bus errors
	 * as the device heats up.
	 */

	temp_clk = __raw_readl( MX51_IO_ADDRESS(MX51_CCM_BASE_ADDR)+ MXC_CCM_CBCDR );

	__raw_writel( (temp_clk & (~0x1c00000)) | bitfield(22, 3, emi_slow_podf),
		      MX51_IO_ADDRESS(MX51_CCM_BASE_ADDR)+ MXC_CCM_CBCDR );

	if (gpio_request(event_gpio_num, "xillybus")) {
		printk(KERN_ERR THIS "GPIO pin %i is already in use\n", event_gpio_num);
		return -ENODEV;
	}

	gpio_direction_input(event_gpio_num);
  
	GCR1 = 0x0111008f |
		bitfield(28, 4, PSZ) |
		bitfield(23, 1, AUS) |
		bitfield(20, 3, CSREC) |
		bitfield(14, 2, BCS) |
		bitfield(12, 2, BCD) |
		bitfield(11, 1, WC) |
		bitfield(8, 3, BL) |
		bitfield(5, 1, RFL) |
		bitfield(4, 1, WFL);
    
	GCR2 = bitfield(0, 2, ADH);

	RCR1 = 
		bitfield(24, 6, RWSC) |
		bitfield(20, 3, ADVA) |
		bitfield(16, 3, ADVN) |
		bitfield(12, 3, OEA) |
		bitfield(4, 3, CSA);

	RCR2 = 
		bitfield(8, 2, RL) |
		bitfield(4, 3, BEA) |
		bitfield(3, 1, BE);

	WCR1 =
		bitfield(30, 1, !BE) |
		bitfield(24, 6, WWSC) |
		bitfield(21, 3, ADVA) |
		bitfield(18, 3, ADVN) |
		bitfield(15, 3, BEA) |
		bitfield(9, 3, WEA) |
		bitfield(3, 3, CSA);

	WEIMCR =
		bitfield(5, 1, INTPOL) |
		bitfield(4, 1, INTEN) |
		bitfield(1, 2, GBCD) |
		bitfield(0, 1, BCM);

	writereg(0x30, GCR1);
	writereg(0x34, GCR2);
	writereg(0x38, RCR1);
	writereg(0x3c, RCR2);
	writereg(0x40, WCR1);
	writereg(0x90, WEIMCR);

	if  (!request_mem_region(MX51_CS2_BASE_ADDR, SZ_64K, "xillybus_sdma")) {
		printk(KERN_ERR THIS "request_mem_region failed. Aborting.\n");
		return -ENODEV;
	}

	cs2_base = ioremap_nocache(MX51_CS2_BASE_ADDR, SZ_64K);

	if (!cs2_base) {
		printk(KERN_WARNING THIS "Failed to obtain I/O space\n");
		ret = -ENODEV;
		goto failed_ioremap;
	}

	return 0;

failed_ioremap:
	release_mem_region(MX51_CS2_BASE_ADDR, SZ_64K);

	return ret;
}

static inline u32 chnenbl_ofs(struct sdma_engine *sdma, unsigned int event)
{
	u32 chnenbl0 = (sdma->version == 2 ? SDMA_CHNENBL0_V2 : SDMA_CHNENBL0_V1);

	return chnenbl0 + event * 4;
}



/*
 * Welcome to the most confusing function in the world. To begin with, it
 * has nothing to do with ownerships. Second, the {event,mcu,dsp}_override
 * arguments are merely the logical INVERSE to EO(i), HO(i) and DO(i),
 * respectively, as they appear in the reference manual, section 52.4.3.5
 * (and i is the channel index).
 * It follows that for mx51, dsp_override must always be false. It also
 * follows that if event_override is true, events are *NOT* ignored, so
 * in a sense, this means that the channel is controlled by events. Same
 * goes for mcu_override for host. So in a sense, the element which is true
 * marks who controls the channel. Which kinda explains the function's name.
 */

static int sdma_config_ownership(struct sdma_channel *sdmac,
				 bool event_override, bool mcu_override, bool dsp_override)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 evt, mcu, dsp;

	if (event_override && mcu_override && dsp_override)
		return -EINVAL;

	evt = __raw_readl(sdma->regs + SDMA_H_EVTOVR);
	mcu = __raw_readl(sdma->regs + SDMA_H_HOSTOVR);
	dsp = __raw_readl(sdma->regs + SDMA_H_DSPOVR);

	if (dsp_override)
		dsp &= ~(1 << channel);
	else
		dsp |= (1 << channel);

	if (event_override)
		evt &= ~(1 << channel);
	else
		evt |= (1 << channel);

	if (mcu_override)
		mcu &= ~(1 << channel);
	else
		mcu |= (1 << channel);

	__raw_writel(evt, sdma->regs + SDMA_H_EVTOVR);
	__raw_writel(mcu, sdma->regs + SDMA_H_HOSTOVR);
	__raw_writel(dsp, sdma->regs + SDMA_H_DSPOVR);

	return 0;
}

/*
 * sdma_run_channel - run a channel and wait till it's done
 */
static int sdma_run_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int ret;

	init_completion(&sdmac->done);

	__raw_writel(1 << channel, sdma->regs + SDMA_H_START);

	ret = wait_for_completion_timeout(&sdmac->done, HZ);

	return ret ? 0 : -ETIMEDOUT;
}

static int sdma_write_datamem(struct sdma_engine *sdma, void *buf, int size,
			      u32 address)
{
	struct sdma_buffer_descriptor *bd0 = sdma->channel[0].bd;
	void *buf_virt;
	dma_addr_t buf_phys;
	int ret;

	buf_virt = dma_alloc_coherent(NULL,
				      size,
				      &buf_phys, GFP_KERNEL);
	if (!buf_virt)
		return -ENOMEM;

	bd0->mode.command = C0_SETDM;
	bd0->mode.count = size / 4;
	bd0->mode.status = BD_DONE | BD_INTR | BD_WRAP | BD_EXTD;
	bd0->buffer_addr = buf_phys;
	bd0->ext_buffer_addr = address;

	memcpy(buf_virt, buf, size);

	ret = sdma_run_channel(&sdma->channel[0]);

	dma_free_coherent(NULL, size, buf_virt, buf_phys);

	return ret;
}


static void sdma_event_enable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 val;
	u32 chnenbl = chnenbl_ofs(sdma, event);

	val = __raw_readl(sdma->regs + chnenbl);
	val |= (1 << channel);
	__raw_writel(val, sdma->regs + chnenbl);
}

static void sdma_event_disable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 chnenbl = chnenbl_ofs(sdma, event);
	u32 val;

	val = __raw_readl(sdma->regs + chnenbl);
	val &= ~(1 << channel);
	__raw_writel(val, sdma->regs + chnenbl);
}


static void mxc_sdma_handle_channel_normal(struct sdma_channel *sdmac)
{
	struct sdma_buffer_descriptor *bd;
	int i, error = 0;

	/*
	 * non loop mode. Iterate over all descriptors, collect
	 * errors and call callback function
	 */
	for (i = 0; i < sdmac->num_bd; i++) {
		bd = &sdmac->bd[i];

		if (bd->mode.status & (BD_DONE | BD_RROR))
			error = -EIO;
	}

	if (error)
		sdmac->status = DMA_ERROR;
	else
		sdmac->status = DMA_SUCCESS;

	if (sdmac->desc.callback)
		sdmac->desc.callback(sdmac->desc.callback_param);
	sdmac->last_completed = sdmac->desc.cookie;
}

static void mxc_sdma_handle_channel(struct sdma_channel *sdmac)
{
	complete(&sdmac->done);

	/* not interested in channel 0 interrupts */
	if (sdmac->channel == 0)
		return;

	mxc_sdma_handle_channel_normal(sdmac);
}

static irqreturn_t sdma_int_handler(int irq, void *dev_id)
{
	struct sdma_engine *sdma = dev_id;
	u32 stat;

	stat = __raw_readl(sdma->regs + SDMA_H_INTR);
	__raw_writel(stat, sdma->regs + SDMA_H_INTR);

	while (stat) {
		int channel = fls(stat) - 1;
		struct sdma_channel *sdmac = &sdma->channel[channel];

		mxc_sdma_handle_channel(sdmac);

		stat &= ~(1 << channel);
	}
	return IRQ_HANDLED;
}

/*
 * sets the pc of SDMA script according to the peripheral type
 */
static void sdma_get_pc(struct sdma_channel *sdmac,
			enum sdma_peripheral_type peripheral_type)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int per_2_emi = 0, emi_2_per = 0;
	/*
	 * These are needed once we start to support transfers between
	 * two peripherals or memory-to-memory transfers
	 */
	int per_2_per = 0, emi_2_emi = 0;

	sdmac->pc_from_device = 0;
	sdmac->pc_to_device = 0;

	switch (peripheral_type) {
	case IMX_DMATYPE_MEMORY:
		emi_2_emi = sdma->script_addrs->ap_2_ap_addr;
		break;
	case IMX_DMATYPE_DSP:
		emi_2_per = sdma->script_addrs->bp_2_ap_addr;
		per_2_emi = sdma->script_addrs->ap_2_bp_addr;
		break;
	case IMX_DMATYPE_FIRI:
		per_2_emi = sdma->script_addrs->firi_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_firi_addr;
		break;
	case IMX_DMATYPE_UART:
		per_2_emi = sdma->script_addrs->uart_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_UART_SP:
		per_2_emi = sdma->script_addrs->uartsh_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ATA:
		per_2_emi = sdma->script_addrs->ata_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ata_addr;
		break;
	case IMX_DMATYPE_CSPI:
	case IMX_DMATYPE_EXT:
	case IMX_DMATYPE_SSI:
		per_2_emi = sdma->script_addrs->app_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_SSI_SP:
	case IMX_DMATYPE_MMC:
	case IMX_DMATYPE_SDHC:
	case IMX_DMATYPE_CSPI_SP:
	case IMX_DMATYPE_ESAI:
	case IMX_DMATYPE_MSHC_SP:
		per_2_emi = sdma->script_addrs->shp_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ASRC:
		per_2_emi = sdma->script_addrs->asrc_2_mcu_addr;
		emi_2_per = sdma->script_addrs->asrc_2_mcu_addr;
		per_2_per = sdma->script_addrs->per_2_per_addr;
		break;
	case IMX_DMATYPE_MSHC:
		per_2_emi = sdma->script_addrs->mshc_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_mshc_addr;
		break;
	case IMX_DMATYPE_CCM:
		per_2_emi = sdma->script_addrs->dptc_dvfs_addr;
		break;
	case IMX_DMATYPE_SPDIF:
		per_2_emi = sdma->script_addrs->spdif_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_spdif_addr;
		break;
	case IMX_DMATYPE_IPU_MEMORY:
		emi_2_per = sdma->script_addrs->ext_mem_2_ipu_addr;
		break;
	default:
		break;
	}

	sdmac->pc_from_device = per_2_emi;
	sdmac->pc_to_device = emi_2_per;
}

static int sdma_load_context(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int load_address;
	struct sdma_context_data *context = sdma->context;
	struct sdma_buffer_descriptor *bd0 = sdma->channel[0].bd;
	int ret;

	if (sdmac->direction == DMA_FROM_DEVICE) {
		load_address = sdmac->pc_from_device;
	} else {
		load_address = sdmac->pc_to_device;
	}

	if (load_address < 0)
		return load_address;

	dev_dbg(sdma->dev, "load_address = %d\n", load_address);
	dev_dbg(sdma->dev, "wml = 0x%08x\n", sdmac->watermark_level);
	dev_dbg(sdma->dev, "shp_addr = 0x%08x\n", sdmac->shp_addr);
	dev_dbg(sdma->dev, "per_addr = 0x%08x\n", sdmac->per_addr);
	dev_dbg(sdma->dev, "event_mask0 = 0x%08x\n", sdmac->event_mask0);
	dev_dbg(sdma->dev, "event_mask1 = 0x%08x\n", sdmac->event_mask1);

	memset(context, 0, sizeof(*context));
	context->channel_state.pc = load_address;

	/* Send by context the event mask,base address for peripheral
	 * and watermark level
	 */
	context->gReg[0] = sdmac->event_mask1;
	context->gReg[1] = sdmac->event_mask0;
	context->gReg[2] = sdmac->per_addr;
	context->gReg[6] = sdmac->shp_addr;
	context->gReg[7] = sdmac->watermark_level;

	bd0->mode.command = C0_SETDM;
	bd0->mode.status = BD_DONE | BD_INTR | BD_WRAP | BD_EXTD;
	bd0->mode.count = sizeof(*context) / 4;
	bd0->buffer_addr = sdma->context_phys;
	bd0->ext_buffer_addr = 2048 + (sizeof(*context) / 4) * channel;

	ret = sdma_run_channel(&sdma->channel[0]);

	return ret;
}

static void sdma_disable_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	__raw_writel(1 << channel, sdma->regs + SDMA_H_STATSTOP);
	sdmac->status = DMA_ERROR;
}

static int sdma_config_channel(struct sdma_channel *sdmac)
{
	int ret;

	sdma_disable_channel(sdmac);

	sdmac->event_mask0 = 0;
	sdmac->event_mask1 = 0;
	sdmac->shp_addr = 0;
	sdmac->per_addr = 0;

	if (sdmac->event_id0) {
		if (sdmac->event_id0 > 32)
			return -EINVAL;
		sdma_event_enable(sdmac, sdmac->event_id0);
	}

	switch (sdmac->peripheral_type) {
	case IMX_DMATYPE_DSP:
		sdma_config_ownership(sdmac, false, true, true);
		break;
	case IMX_DMATYPE_MEMORY:
		sdma_config_ownership(sdmac, false, true, false);
		break;
	default:
		sdma_config_ownership(sdmac, true, true, false);
		break;
	}

	sdma_get_pc(sdmac, sdmac->peripheral_type);

	if ((sdmac->peripheral_type != IMX_DMATYPE_MEMORY) &&
	    (sdmac->peripheral_type != IMX_DMATYPE_DSP)) {
		/* Handle multiple event channels differently */
		if (sdmac->event_id1) {
			sdmac->event_mask1 = 1 << (sdmac->event_id1 % 32);
			if (sdmac->event_id1 > 31)
				sdmac->watermark_level |= 1 << 31;
			sdmac->event_mask0 = 1 << (sdmac->event_id0 % 32);
			if (sdmac->event_id0 > 31)
				sdmac->watermark_level |= 1 << 30;
		} else {
			sdmac->event_mask0 = 1 << sdmac->event_id0;
			sdmac->event_mask1 = 1 << (sdmac->event_id0 - 32);
		}
		/* Watermark Level */
		sdmac->watermark_level |= sdmac->watermark_level;
		/* Address */
		sdmac->shp_addr = sdmac->per_address;
	} else {
		sdmac->watermark_level = 0; /* FIXME: M3_BASE_ADDRESS */
	}

	ret = sdma_load_context(sdmac);

	return ret;
}

static int sdma_set_channel_priority(struct sdma_channel *sdmac,
				     unsigned int priority)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	if (priority < MXC_SDMA_MIN_PRIORITY
	    || priority > MXC_SDMA_MAX_PRIORITY) {
		return -EINVAL;
	}

	__raw_writel(priority, sdma->regs + SDMA_CHNPRI_0 + 4 * channel);

	return 0;
}

static int sdma_request_channel(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int ret = -EBUSY;

	sdmac->bd = dma_alloc_coherent(NULL, PAGE_SIZE, &sdmac->bd_phys, GFP_KERNEL);
	if (!sdmac->bd) {
		ret = -ENOMEM;
		goto out;
	}

	memset(sdmac->bd, 0, PAGE_SIZE);

	sdma->channel_control[channel].base_bd_ptr = sdmac->bd_phys;
	sdma->channel_control[channel].current_bd_ptr = sdmac->bd_phys;

	clk_enable(sdma->clk);

	sdma_set_channel_priority(sdmac, MXC_SDMA_DEFAULT_PRIORITY);

	init_completion(&sdmac->done);

	sdmac->buf_tail = 0;

	return 0;
out:

	return ret;
}

static struct sdma_channel *to_sdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sdma_channel, chan);
}

static int sdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct imx_dma_data *data = chan->private;
	int prio, ret;

	if (!data)
		return -EINVAL;

	switch (data->priority) {
	case DMA_PRIO_HIGH:
		prio = 3;
		break;
	case DMA_PRIO_MEDIUM:
		prio = 2;
		break;
	case DMA_PRIO_LOW:
	default:
		prio = 1;
		break;
	}

	sdmac->peripheral_type = data->peripheral_type;
	sdmac->event_id0 = data->dma_request;
	ret = sdma_set_channel_priority(sdmac, prio);
	if (ret)
		return ret;

	ret = sdma_request_channel(sdmac);
	if (ret)
		return ret;

	return 0;
}

static void sdma_free_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;

	sdma_disable_channel(sdmac);

	if (sdmac->event_id0)
		sdma_event_disable(sdmac, sdmac->event_id0);
	if (sdmac->event_id1)
		sdma_event_disable(sdmac, sdmac->event_id1);

	sdmac->event_id0 = 0;
	sdmac->event_id1 = 0;

	sdma_set_channel_priority(sdmac, 0);

	dma_free_coherent(NULL, PAGE_SIZE, sdmac->bd, sdmac->bd_phys);

	clk_disable(sdma->clk);
}


static int sdma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
			unsigned long arg)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct dma_slave_config *dmaengine_cfg = (void *)arg;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		sdma_disable_channel(sdmac);
		return 0;
	case DMA_SLAVE_CONFIG:
		if (dmaengine_cfg->direction == DMA_FROM_DEVICE) {
			sdmac->per_address = dmaengine_cfg->src_addr;
			sdmac->watermark_level = dmaengine_cfg->src_maxburst;
			sdmac->word_size = dmaengine_cfg->src_addr_width;
		} else {
			sdmac->per_address = dmaengine_cfg->dst_addr;
			sdmac->watermark_level = dmaengine_cfg->dst_maxburst;
			sdmac->word_size = dmaengine_cfg->dst_addr_width;
		}
		return sdma_config_channel(sdmac);
	default:
		return -ENOSYS;
	}

	return -EINVAL;
}

static enum dma_status sdma_tx_status(struct dma_chan *chan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *txstate)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	dma_cookie_t last_used;

	last_used = chan->cookie;

	dma_set_tx_state(txstate, sdmac->last_completed, last_used, 0);

	return sdmac->status;
}

static void sdma_issue_pending(struct dma_chan *chan)
{
	/*
	 * Nothing to do. We only have a single descriptor
	 */
}

#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1	34

static int __init sdma_init(struct sdma_engine *sdma)
{
	int i, ret;
	dma_addr_t ccb_phys;

	switch (sdma->version) {
	case 1:
		sdma->num_events = 32;
		break;
	case 2:
		sdma->num_events = 48;
		break;
	default:
		dev_err(sdma->dev, "Unknown version %d. aborting\n", sdma->version);
		return -ENODEV;
	}

	clk_enable(sdma->clk);

	/* Be sure SDMA has not started yet */
	__raw_writel(0, sdma->regs + SDMA_H_C0PTR);

	sdma->channel_control = dma_alloc_coherent(NULL,
						   MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control) +
						   sizeof(struct sdma_context_data),
						   &ccb_phys, GFP_KERNEL);

	if (!sdma->channel_control) {
		ret = -ENOMEM;
		goto err_dma_alloc;
	}

	sdma->context = (void *)sdma->channel_control +
		MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control);
	sdma->context_phys = ccb_phys +
		MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control);

	/* Zero-out the CCB structures array just allocated */
	memset(sdma->channel_control, 0,
	       MAX_DMA_CHANNELS * sizeof (struct sdma_channel_control));

	/* disable all channels */
	for (i = 0; i < sdma->num_events; i++)
		__raw_writel(0, sdma->regs + chnenbl_ofs(sdma, i));

	/* All channels have priority 0 */
	for (i = 0; i < MAX_DMA_CHANNELS; i++)
		__raw_writel(0, sdma->regs + SDMA_CHNPRI_0 + i * 4);

	ret = sdma_request_channel(&sdma->channel[0]);
	if (ret)
		goto err_dma_alloc;

	sdma_config_ownership(&sdma->channel[0], false, true, false);

	/* Set Command Channel (Channel Zero) */
	__raw_writel(0x4050, sdma->regs + SDMA_CHN0ADDR);

	/* Set bits of CONFIG register but with static context switching */
	/* FIXME: Check whether to set ACR bit depending on clock ratios */
	__raw_writel(0, sdma->regs + SDMA_H_CONFIG);

	__raw_writel(ccb_phys, sdma->regs + SDMA_H_C0PTR);

	/* Set bits of CONFIG register with given context switching mode */
	__raw_writel(SDMA_H_CONFIG_CSM, sdma->regs + SDMA_H_CONFIG);

	/* Initializes channel's priorities */
	sdma_set_channel_priority(&sdma->channel[0], 7);

	clk_disable(sdma->clk);

	return 0;

err_dma_alloc:
	clk_disable(sdma->clk);
	dev_err(sdma->dev, "initialisation failed with %d\n", ret);
	return ret;
}

void sdma_irq_callback(void *param) {
	if (xillybus_handler)
		xillybus_handler(0, xillybus_userdata);
}

static int setup_fpga_interface(struct sdma_engine *sdma)
{
	const int channel = 1;

	struct sdma_channel *sdmac = &sdma->channel[channel];
	const u32 sdma_code[24] = {
		0x6c20672b, 0x07647d02, 0x04007cfa, 0x612b622b, 0x662b0762, 0x7d0e6900, 0x6d0406da, 0x7d056e18,
		0x000002a6, 0x0e087cf8, 0x6a186c14, 0x6c2b7ce8, 0x7de7632b, 0x6904008f, 0x38037802, 0x6b290312,
		0x4a007d09, 0x6d0006da, 0x7d056e18, 0x000002a6, 0x0e087cf8, 0x6a180763, 0x7ce80300, 0x7de60000,
	};

	const int origin = 0xe00; /* In data space terms (32 bits/address) */

	struct sdma_context_data *context = sdma->context;

	int ret;

	ret = eim_init();

	if (ret) {
		printk(KERN_ERR THIS "Failed to initialize EIM bus\n");
		return ret;
	}

	sdma_write_datamem(sdma, (void *) sdma_code, sizeof(sdma_code), origin);

	ret = sdma_request_channel(sdmac);

	if (ret) {
		printk(KERN_ERR "Failed to request channel\n");
		return ret;
	}
  
	sdma_disable_channel(sdmac);

	/* Don't let events run yet: */
	sdma_config_ownership(sdmac, true, true, false);

	memset(context, 0, sizeof(*context));

	context->channel_state.pc = origin * 2; /* In program space addressing */
	context->gReg[4] = MX51_CS2_BASE_ADDR + 0x80;  /* Request region */
	context->gReg[5] = MX51_CS2_BASE_ADDR + 0x8000; /* Data region */

	ret = sdma_write_datamem(sdma, (void *) context, sizeof(*context),
				 0x800 + (sizeof(*context) / 4) * channel);
  
	if (ret) {
		printk(KERN_ERR "Failed to load context\n");
		return ret;
	}

	sdmac->desc.callback = sdma_irq_callback;
	sdmac->desc.callback_param = NULL;

	xillybus_sdmac = sdmac;
  
	return 0; /* Success! */
}

static int __init sdma_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	struct resource *iores;
	struct sdma_platform_data *pdata = pdev->dev.platform_data;
	int i;
	struct sdma_engine *sdma;

	sdma = kzalloc(sizeof(*sdma), GFP_KERNEL);
	if (!sdma)
		return -ENOMEM;

	sdma->dev = &pdev->dev;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!iores || irq < 0 || !pdata) {
		ret = -EINVAL;
		goto err_irq;
	}

	if (!request_mem_region(iores->start, resource_size(iores), pdev->name)) {
		ret = -EBUSY;
		goto err_request_region;
	}

	sdma->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(sdma->clk)) {
		ret = PTR_ERR(sdma->clk);
		goto err_clk;
	}

	sdma->regs = ioremap(iores->start, resource_size(iores));
	if (!sdma->regs) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = request_irq(irq, sdma_int_handler, 0, "xillybus_sdma", sdma);
	if (ret)
		goto err_request_irq;

	sdma->script_addrs = kzalloc(sizeof(*sdma->script_addrs), GFP_KERNEL);
	if (!sdma->script_addrs)
		goto err_alloc;

	sdma->version = pdata->sdma_version;

	dma_cap_set(DMA_SLAVE, sdma->dma_device.cap_mask);
	dma_cap_set(DMA_CYCLIC, sdma->dma_device.cap_mask);

	INIT_LIST_HEAD(&sdma->dma_device.channels);
	/* Initialize channel parameters */
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		struct sdma_channel *sdmac = &sdma->channel[i];

		sdmac->sdma = sdma;
		spin_lock_init(&sdmac->lock);

		sdmac->chan.device = &sdma->dma_device;
		sdmac->channel = i;

		/*
		 * Add the channel to the DMAC list. Do not add channel 0 though
		 * because we need it internally in the SDMA driver. This also means
		 * that channel 0 in dmaengine counting matches sdma channel 1.
		 */
		if (i)
			list_add_tail(&sdmac->chan.device_node,
				      &sdma->dma_device.channels);
	}

	ret = sdma_init(sdma);
	if (ret)
		goto err_init;

	sdma->dma_device.dev = &pdev->dev;

	sdma->dma_device.device_alloc_chan_resources = sdma_alloc_chan_resources;
	sdma->dma_device.device_free_chan_resources = sdma_free_chan_resources;
	sdma->dma_device.device_tx_status = sdma_tx_status;
	sdma->dma_device.device_control = sdma_control;
	sdma->dma_device.device_issue_pending = sdma_issue_pending;
	sdma->dma_device.dev->dma_parms = &sdma->dma_parms;
	dma_set_max_seg_size(sdma->dma_device.dev, 65535);

	xillybus_handler = NULL; /* Just to be sure */
	xillybus_dev = sdma->dev;

	if ((ret = setup_fpga_interface(sdma))) {
		printk(KERN_ERR THIS "setup_fpga_interface() failed!\n");
		goto err_init;
	}

	return 0;
	
err_init:
	kfree(sdma->script_addrs);
err_alloc:
	free_irq(irq, sdma);
err_request_irq:
	iounmap(sdma->regs);
err_ioremap:
	clk_put(sdma->clk);
err_clk:
	release_mem_region(iores->start, resource_size(iores));
err_request_region:
err_irq:
	kfree(sdma);
	return ret;
}

static int __exit sdma_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static struct platform_driver sdma_driver = {
	.driver		= {
		.name	= "imx-sdma",
	},
	.remove		= __exit_p(sdma_remove),
};

static int __init sdma_module_init(void)
{
	return platform_driver_probe(&sdma_driver, sdma_probe);
}
module_init(sdma_module_init);

struct device *sdma_get_dev(void) {
	return xillybus_dev;
}
EXPORT_SYMBOL(sdma_get_dev);

void *sdma_xillybus_init(void *userdata, irq_handler_t handler)
{
	xillybus_userdata = userdata;
	xillybus_handler = handler;

	sdma_event_enable(xillybus_sdmac, 15); /* Connect to channel 15 */
	sdma_config_ownership(xillybus_sdmac, true, false, false);

	return cs2_base;
}
EXPORT_SYMBOL(sdma_xillybus_init);

void sdma_xillybus_remove(void)
{
	sdma_event_disable(xillybus_sdmac, 15); /* Connect to channel 15 */
	sdma_disable_channel(xillybus_sdmac);
	xillybus_handler = NULL;
}
EXPORT_SYMBOL(sdma_xillybus_remove);

void *sdma_get_userdata(void)
{
	return xillybus_userdata;
}
EXPORT_SYMBOL(sdma_get_userdata);

MODULE_AUTHOR("Eli Billauer, Xillybus Ltd");
MODULE_DESCRIPTION("Xillybus SDMA interface driver");
MODULE_LICENSE("GPL");
