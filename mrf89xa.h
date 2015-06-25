#ifndef __MRF89XA_H
#define __MRF89XA_H

/*
  http://elinux.org/RPi_BCM2835_GPIOs
*/
#define CSCON_PIN 23
#define DATA_PIN  25

#define CMD_READ_REGISTER(N) (0x40 | (N << 1))

#endif /* __MRF89XA_H */
