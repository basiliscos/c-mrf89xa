#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/spi/spi.h>

#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "mrf89xa.h"

#define MRFSPI_DRV_NAME "mrfspi"
#define MRFSPI_DRV_VERSION "0.1"

long mrf_ioctl_unlocked(struct file *filp, unsigned int cmd, unsigned long arg);
static int write_register(u8 index, u8 value);
static u8 read_register(u8 index);

static int ignore_registers = 0;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ivan Baidakou");
MODULE_DESCRIPTION("Microchip MRF89XA SPI Driver");
MODULE_VERSION(MRFSPI_DRV_VERSION);

module_param(ignore_registers, int, S_IRUGO);
MODULE_PARM_DESC(ignore_registers, "Ignore initial register values on probe.");


struct mrf_dev {
  struct cdev cdev;
  struct semaphore device_semaphore;
  uint32_t state;

  struct semaphore operation_semaphore;
  struct spi_device *spi;
  struct gpio_desc *config_pin;
  struct gpio_desc *data_pin;
  struct proc_dir_entry* proc_file;
};

struct mrf_dev *mrf_device = NULL;

static u8 por_register_values[] = { 0x28, 0x88, 0x03, 0x07, 0x0C, 0x0F };

static u8 default_register_values[] = {
  /* 0 == REG_GCON         */ CHIPMODE_STBYMODE | FREQBAND_950_863 | VCO_TRIM_11,
  /* 1 == REG_DMOD         */ MODSEL_FSK | DATAMODE_PACKET | IFGAIN_0,
  /* 1 == REG_FDEV         */ FREGDEV_80,
  /* 3 == REG_BRS          */ BITRATE_25,
  /* 4 == REG_FLTH         */ 0 /* unused with FSK-modulation */,
  /* 5 == REG_FIFO         */ FIFOSIZE_64,
  /* 6 == REG_R1C          */ 0 /* frequency will be defined later */,
  /* 7 == REG_P1C          */ 0 /* frequency will be defined later */,
  /* 8 == REG_S1C          */ 0 /* frequency will be defined later */,
  /* 9, unused             */ 0,
  /* 10, unused            */ 0,
  /* 11, unused            */ 0,
  /* 12 == REG_PAC         */ 0, /* unused with FSK-modulation  */
  /* 13 == REG_FTXRXI      */ IRQ0_RX_STDBY_SYNCADRS | IRQ1_RX_STDBY_CRCOK | IRQ1_TX_TXDONE,
  /* 14 == REG_FTPRI       */ DEF_IRQPARAM1 | IRQ0_TX_START_FIFONOTEMPTY | IRQ1_PLL_LOCK_PIN_ON,
  /* 15, unused            */ 0,
  /* 16 == REG_FILC        */ PASSIVEFILT_378 | RXFC_FOPLUS100,
  /* 17 == REG_PFC         */ FO_100,
  /* 18 == REG_SYNC        */ SYNC_SIZE_32 | SYNC_ON | SYNC_ERRORS_0,
  /* 19, reserved          */ DEF_RXPARAM3,
  /* 20, r/0               */ 0,
  /* 21, unused            */ 0 /* unused with FSK-modulation */,
  /* 22 == REG_SYNC_WORD_1 */ 0 /* network will be defined later */,
  /* 23 == REG_SYNC_WORD_2 */ 0 /* network will be defined later */,
  /* 24 == REG_SYNC_WORD_3 */ 0 /* network will be defined later */,
  /* 25 == REG_SYNC_WORD_4 */ 0 /* network will be defined later */,
  /* 26 == REG_TXCON       */ FC_400 | MRF_TXPOWER_PLUS_13,
  /* 27 == REG_CLKOUT      */ CLKOUT_OFF /* not needed*/,
  /* 28 == REG_PLOAD       */ MANCHESTER_OFF | PAYLOAD_64,
  /* 29 == REG_NADDS       */ 0 /* node address will be defined later */,
  /* 30 == REG_PKTC        */ PKT_FORMAT_VARIABLE | PREAMBLE_SIZE_4 | WHITENING_OFF | CRC_ON | ADRSFILT_ME_AND_00_AND_FF,
  /* 31 == REG_FCRC        */ FIFO_AUTOCLR_ON | FIFO_STBY_ACCESS_WRITE,
};

static int mrf_open(struct inode *inode, struct file *filp) {
  int status;

  printk(KERN_INFO "mrf: open device %p\n", mrf_device);

  down(&mrf_device->device_semaphore);

  if (! (mrf_device->state & MRF_STATE_DEVICEOPENED) ) {
    try_module_get(THIS_MODULE);
    mrf_device->state = MRF_STATE_DEVICEOPENED;
    status = 0; /* success */
  } else {
    status = -EBUSY;
  }

  up(&mrf_device->device_semaphore);
  return status;
}

static int mrf_release(struct inode *inode, struct file *filp) {
  printk(KERN_INFO "mrf: release device\n");
  down(&mrf_device->device_semaphore);

  module_put(THIS_MODULE);
  mrf_device->state &= ~(MRF_STATE_DEVICEOPENED);

  up(&mrf_device->device_semaphore);
  return 0;
}

static ssize_t mrf_write(struct file *filp, const char *buff, size_t length, loff_t * offset) {
  printk(KERN_INFO "mrf: write to device isn't supported yet\n");
  return -EINVAL;
}

static ssize_t mrf_read(struct file *filp, char *buff, size_t length, loff_t *offset) {
  int bytes_read = 0;
  /* printk(KERN_INFO "mrf: reading from device\n"); */

  /* if (length && mrf_dev->counter < MRFSPI_MAX_COUNTER) { */
  /*   put_user('a', buff++); */
  /*   length--; */
  /*   bytes_read++; */
  /*   mrf_dev->counter++; */
  /* } */
  return bytes_read;
}

static int mrf_dump_stats(struct seq_file *m, void *v){
  int i, j;
  down(&mrf_device->device_semaphore);

  /* print all 32 registers, 4 per row */
  seq_printf(m, "mrf registers dump\n");
  for (i = 0; i < 8; i++){
    for (j = 0; j < 4; j++) {
      int index = i*4 + j;
      u8 value = read_register(index);
      seq_printf(m, "register %02d = 0x%.2x %s", index + 1, value, (j == 3) ? "\n" : "| ");
    }
  }

  up(&mrf_device->device_semaphore);
  return 0;
}

static int mrf_proc_open(struct inode *inode, struct file *file) {
  return single_open(file, mrf_dump_stats, NULL);
}

static struct file_operations mrf_fops = {
  .read           = mrf_read,
  .write          = mrf_write,
  .open           = mrf_open,
  .release        = mrf_release,
  .unlocked_ioctl = mrf_ioctl_unlocked,
};

static struct file_operations mrf_proc_fops = {
  .owner          = THIS_MODULE,
  .open           = mrf_proc_open,
  .read           = seq_read,
  .llseek	      = seq_lseek,
  .release	      = single_release,
};

static int write_register(u8 index, u8 value) {
  u8 buff[] = {index << 1, value};
  gpiod_set_value(mrf_device->config_pin, 0);
  gpiod_set_value(mrf_device->data_pin, 0);
  return spi_write(mrf_device->spi, buff, ARRAY_SIZE(buff));
}

static u8 read_register(u8 index) {
  return spi_w8r8(mrf_device->spi, CMD_READ_REGISTER(index));
}

static int initialize_registers(void) {
  int i;
  int status;
  printk(KERN_INFO "mrf: initializing registers\n");
  for (i = 0; i < ARRAY_SIZE(default_register_values); i++) {
    status = write_register((u8)i, default_register_values[i]);
    if (status) goto err;
  }
  printk(KERN_INFO "mrf: registers initialized\n");
  return 0; /* success */
 err:
  printk(KERN_INFO "mrf: registers initialization failed\n");
  return status;
}


long mrf_ioctl_unlocked(struct file *filp, unsigned int cmd, unsigned long arg) {
  int status = 0;
#define write_register_protected(ADDR, VALUE) status = write_register((ADDR), (VALUE)); if (status) goto finish;
  struct gpio_desc *reset_pin;

  printk(KERN_INFO "mrf: ioctl (%d)\n", cmd);

  if (_IOC_TYPE(cmd) != MRF_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > MRF_IOC_MAXNR) return -ENOTTY;

  if (_IOC_DIR(cmd) & _IOC_READ)
    status = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
  else if (_IOC_DIR(cmd) & _IOC_WRITE)
    status = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
  if (status) return -EFAULT;

  down(&mrf_device->operation_semaphore);
  printk(KERN_INFO "mrf: ioctl switch\n");
  switch(cmd) {
  case MRF_IOC_RESET:
    reset_pin = gpio_to_desc(RESET_PIN);
    if (IS_ERR_OR_NULL(reset_pin)) {
      printk(KERN_INFO "mrf: reset pin (%d) not found", RESET_PIN);
      status = -EFAULT;
      goto finish;
    }
    gpiod_direction_output(reset_pin, 1);
    gpiod_set_value(reset_pin, 1);
    /* wait until mrf device reset */
    msleep(RESET_DELAY);
    gpiod_set_value(reset_pin, 0);
    gpiod_put(reset_pin);
    /* wait until mrf device reset */
    msleep(RESET_DELAY);
    status = initialize_registers();
    if (status) goto finish;
    printk(KERN_INFO "mrf: device reset successfull\n");
    break;
  case MRF_IOC_SETADDR:
    {
      mrf_address *addr = (mrf_address*) arg;
      uint8_t *network_parts = (uint8_t*) &addr->network_id;
      if (addr->node_id == MRF_BROADCAST_NODEADDR ) {
        status = -ENOTTY;
        goto finish;
      }
      write_register_protected(REG_NADDS, addr->node_id);
      write_register_protected(REG_SYNC_WORD_1, network_parts[0]);
      write_register_protected(REG_SYNC_WORD_2, network_parts[1]);
      write_register_protected(REG_SYNC_WORD_3, network_parts[2]);
      write_register_protected(REG_SYNC_WORD_4, network_parts[3]);
      mrf_device->state |= MRF_STATE_ADDRESSASSIGNED;
    }
    break;
  case MRF_IOC_SETFREQ:
    {
      uint8_t *rps = (uint8_t*) &arg;
      write_register_protected(REG_R1C, *rps++);
      write_register_protected(REG_P1C, *rps++);
      write_register_protected(REG_S1C, *rps++);
      mrf_device->state |= MRF_STATE_FREQASSIGNED;
    }
    break;
  case MRF_IOC_SETPOWER:
    {
      uint8_t power_level = (uint8_t) arg;
      write_register_protected(REG_TXCON, (FC_400 | power_level));
    }
    break;
  default:  /* redundant, as cmd was checked against MAXNR */
    return -ENOTTY;
  };

 finish:
  up(&mrf_device->operation_semaphore);
  printk(KERN_INFO "mrf: ioctl result: %d\n", status);
  return status;
}


static int mrfdev_probe(struct spi_device *spi) {
  u8 i;
  u8 mrf_found = 1;
  int status;

  printk(KERN_INFO "mrf: probing spi device %p for mrf presence\n", spi);

  gpiod_set_value(mrf_device->config_pin, 0);
  gpiod_set_value(mrf_device->data_pin, 0);
  for (i = 0; i < ARRAY_SIZE(por_register_values); i++) {
    u8 got = spi_w8r8(spi, CMD_READ_REGISTER(i));
    u8 expected = por_register_values[i];
    printk(KERN_INFO "mrf: probing %d register. Got: %.2x, expected: %.2x\n", i, got, expected);
    mrf_found &= (got == expected);
  }
  if (mrf_found | ignore_registers) {
    /* success */
    printk(KERN_INFO "mrf: device found\n");
    status = 0;
    mrf_device->state |= MRF_STATE_DEVICEFOUND;
  }
  else {
    status = -ENODEV;
  }
  return status;
}

static int mrfdev_remove(struct spi_device *spi)
{
  printk(KERN_INFO "mrf: dev_remove\n");
  return 0;
}

static struct spi_driver mrfdev_spi_driver = {
	.driver = {
		.name =		"mrf",
		.owner =	THIS_MODULE,
	},
	.probe = mrfdev_probe,
	.remove = mrfdev_remove,
};

static struct spi_board_info mrf_board_info = {
  .modalias = "mrf",
  .max_speed_hz = 1800000,
  .chip_select = 1,
  /*.irq = ? */
  /*.platform_data = ... */
};

static __init int mrf_init(void) {
  int status;
  int driver_registered = 0, cdev_added = 0;
  struct spi_master *master;
  struct spi_device *spi = NULL;
  struct mrf_dev* mrf_dev = NULL;
  dev_t device_id = 0;

  printk(KERN_INFO "mrf: loading module (ignore_registers = %d)\n", ignore_registers);

  /* look up for spi master */
  master = spi_busnum_to_master(MRFSPI_BUS_NO);
  if (!master) {
    printk(KERN_INFO "mrf: spi master not found on bus %d\n", MRFSPI_BUS_NO);
    status = -ENODEV;
    goto err;
  }

  /* allocate memory for mrf device */
  mrf_dev = kzalloc(sizeof(struct mrf_dev), GFP_KERNEL);
  if ( !mrf_dev ) {
    status = -ENOMEM;
    goto err;
  }
  printk(KERN_INFO "mrf: allocated device structure %p\n",mrf_dev);

  /* insert new spi device, which actually could be a proxy */
  spi = spi_new_device(master, &mrf_board_info);
  if (!spi) {
    printk(KERN_INFO "mrf: cannot add new spi device\n");
    status = -ENODEV;
    goto err;
  }
  printk(KERN_INFO "mrf: inserted spi device %p\n", spi);

  /* allocate control and data GPIO pins */
  //mrf_dev->config_pin = gpiod_get(NULL, CSCON_NAME, GPIOD_OUT_LOW);
  mrf_dev->config_pin = gpio_to_desc(CSCON_PIN);
  if (IS_ERR_OR_NULL(mrf_dev->config_pin)) {
    printk(KERN_INFO "mrf: cannot get config_pin(%d)\n", CSCON_PIN);
    status = -ENODEV;
    goto err;
  }
  gpiod_direction_output(mrf_dev->config_pin, 0);
  gpiod_set_value(mrf_dev->config_pin, 0);

  mrf_dev->data_pin = gpio_to_desc(DATA_PIN);
  if (IS_ERR_OR_NULL(mrf_dev->data_pin)) {
    printk(KERN_INFO "mrf: cannot get data_pin(%d)\n", DATA_PIN);
    status = -ENODEV;
    goto err;
  }
  gpiod_direction_output(mrf_dev->data_pin, 0);
  gpiod_set_value(mrf_dev->data_pin, 0);

  /* characted device allocation */
  status = alloc_chrdev_region(&device_id, 1, 1, "mrf");
  if ( status < 0 ) {
    printk(KERN_INFO "mrf: cannot allocate character device\n");
    goto err;
  }
  printk(KERN_INFO "mrf: allocated major device numer: %d\n", MAJOR(device_id));

  /* initialize mrf device */
  sema_init(&mrf_dev->device_semaphore, 1);
  sema_init(&mrf_dev->operation_semaphore, 1);
  cdev_init(&mrf_dev->cdev, &mrf_fops);
  mrf_dev->cdev.owner = THIS_MODULE;
  mrf_dev->cdev.ops = &mrf_fops;
  mrf_dev->spi = spi;
  mrf_dev->state = 0;

  mrf_device = mrf_dev;

  /* add character device */
  printk(KERN_INFO "mrf: adding character device\n");
  status = cdev_add(&mrf_dev->cdev, device_id, 1);
  if (status) {
    printk(KERN_INFO "mrf: cannot add character deviced\n");
    goto err;
  }
  cdev_added = 1;

  /* register and probe mrf device connection via spi */
  status = spi_register_driver(&mrfdev_spi_driver);
  if (status < 0) {
    printk(KERN_INFO "mrf: spi_register_driver failed\n");
    goto err;
  }
  driver_registered = 1;
  /* check that probe succeed */
  if ( !(mrf_device->state & MRF_STATE_DEVICEFOUND)) {
    printk(KERN_INFO "mrf: device hasn't been probed successfully\n");
    status = -ENODEV;
    goto err;
  }
  /* from here spi and spi_device point the same */
  printk(KERN_INFO "mrf: spi device found, max speed = %dKHz, chip select = %d\n",
         mrf_board_info.max_speed_hz, mrf_board_info.chip_select);

  /* create proc fs entry */
  mrf_device->proc_file = proc_create(MRFSPI_DRV_NAME, 0, NULL, &mrf_proc_fops);
  if (! mrf_device->proc_file ) {
    printk(KERN_INFO "mrf: could not initialize /proc/%s\n", MRFSPI_DRV_NAME);
    status = -ENOMEM;
    goto err;
  }

  /* all OK */
  printk(KERN_INFO "mrf: initialization succeed\n");
  return 0;

 err:
  printk(KERN_INFO "mrf: failed\n");
  if (driver_registered) spi_unregister_driver(&mrfdev_spi_driver);
  if (cdev_added) cdev_del(&mrf_device->cdev);
  if (device_id) unregister_chrdev_region(device_id, 1);
  if (mrf_dev && mrf_dev->config_pin && !IS_ERR_OR_NULL(mrf_dev->config_pin)) gpiod_put(mrf_dev->config_pin);
  if (mrf_dev && mrf_dev->data_pin && !IS_ERR_OR_NULL(mrf_dev->data_pin)) gpiod_put(mrf_dev->data_pin);
  if (spi) spi_unregister_device(spi);
  if (mrf_dev) kfree(mrf_dev);
  mrf_device = NULL;
  return status;
}

static void __exit mrf_exit(void) {
  dev_t device_id;
  printk(KERN_INFO "mrf: removing module\n");
  /* TODO: check errors */

  remove_proc_entry(MRFSPI_DRV_NAME, NULL);

  device_id = mrf_device->cdev.dev;

  cdev_del(&mrf_device->cdev);
  printk(KERN_INFO "mrf: character device removed\n");

  unregister_chrdev_region(device_id, 1);
  printk(KERN_INFO "mrf: character device deallocated\n");

  gpiod_put(mrf_device->config_pin);
  gpiod_put(mrf_device->data_pin);

  printk(KERN_INFO "mrf: mrf device memory deallocated\n");

  spi_unregister_driver(&mrfdev_spi_driver);
  printk(KERN_INFO "mrf: spi driver unregistered\n");

  spi_unregister_device(mrf_device->spi);
  printk(KERN_INFO "mrf: spi device unregistered\n");

  kfree(mrf_device);
  mrf_device = NULL;
}


module_init(mrf_init);
module_exit(mrf_exit);
