#ifndef __MRF89XA_H
#define __MRF89XA_H

#include <linux/ioctl.h>

/*
  http://elinux.org/RPi_BCM2835_GPIOs
*/
#define CSCON_PIN 23
#define RESET_PIN 24
#define DATA_PIN  25

/* in microseconds */
#define RESET_DELAY 100
#define RESET_WAIT  6000

#define MRF_IOC_MAGIC 'n'

#define MRF_IOCRESET _IO(MRF_IOC_MAGIC, 0)
#define MRF_IOC_MAXNR 0

/* raspberry pi has just 0-bus */
#define MRFSPI_BUS_NO 0

#define CMD_READ_REGISTER(N) (0x40 | (N << 1))


#endif /* __MRF89XA_H */
