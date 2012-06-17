/*
 * fpga_l3_fade - driver for L3 communication protocol with FPGA based system
 * Copyright (C) 2012 by Wojciech M. Zabolotny
 * Institute of Electronic Systems, Warsaw University of Technology
 *
 *  This code is PUBLIC DOMAIN
 */

#include<termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdio.h>
#include <strings.h>
#include <stdlib.h>
#include <stdint.h>

#include <sys/socket.h>
#include <linux/serial.h>
#include "fpga_l3_fade.h"

void main()
{
  struct l3_v1_buf_pointers bp;
  struct l3_v1_slave sl = {
   .mac = {0xde, 0xad, 0xba, 0xbe, 0xbe,0xef},
   .devname = "eth0"
  };
  int res, blen;
  int data = 0;
  unsigned char * v;
  int frs=open("/dev/l3_fpga0",O_RDONLY);
  blen = ioctl(frs,L3_V1_IOC_GETBUFLEN,NULL);
  res=ioctl(frs,L3_V1_IOC_SETWAKEUP,2000000);
  printf("length of buffer: %d, result of set wakeup: %d\n",blen,res);
  v=(unsigned char *)mmap(0,blen,PROT_READ,MAP_PRIVATE,frs,0);
  if(!v) {
    printf("mmap failed\n");
    exit(1);
  }
  //Start the transmission
  res = ioctl(frs,L3_V1_IOC_STARTMAC,&sl);
  printf("Result of ster: %d\n",res);
  do{
    struct pollfd pfd[1] = {{.fd = frs, .events = POLLIN, .revents = 0}};
    int ptr=0;
    int len=0;
    int pres;
    //Wait for data using "poll"
    pres = poll(pfd,1,-1);
    if(pres<0) {
     perror("Error in poll:");
     exit(1);
    }
    len = ioctl(frs,L3_V1_IOC_READPTRS,&bp);
    printf("len=%d head:%d tail: %d\n",len,bp.head, bp.tail);
    //OK. The data are read, let's analyze them
    while (bp.head != bp.tail)  {
      uint32_t c;
      c = *(uint32_t *)(v+bp.tail);
      c = ntohl(c);
      bp.tail=(bp.tail+1) & (blen-1);
      bp.tail=(bp.tail+1) & (blen-1);
      bp.tail=(bp.tail+1) & (blen-1);
      bp.tail=(bp.tail+1) & (blen-1);
      if(c != data) {
	printf("odebrane: %8.8x oczekiwane: %8.8x \n",c,data);
	exit(1);
      }
      data ++;
    }
    //printf("\n");
    fflush(stdout);
    //Confirm reception of data   
    ioctl(frs,L3_V1_IOC_WRITEPTRS,len);    
    sleep(1);
  } while (1);
  //fo=fopen("data.bin","w");
  //fwrite(v,i,1,fo);
  //fclose(fo);
}
