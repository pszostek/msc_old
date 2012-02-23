#ifndef _XILLYBUS_SDMA_H
#define _XILLYBUS_SDMA_H
#include <linux/interrupt.h>
#include <linux/device.h>

struct device *sdma_get_dev(void);
void *sdma_xillybus_init(void *, irq_handler_t);
void sdma_xillybus_remove(void);
void *sdma_get_userdata(void);
#endif
