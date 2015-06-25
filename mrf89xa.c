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

#include "mrf89xa.h"

#define MRFSPI_DRV_NAME "mrfspi"
#define MRFSPI_DRV_VERSION "0.1"
#define MRFSPI_MAX_COUNTER 10
/* raspberry pi has just 0-bus */
#define MRFSPI_BUS_NO 0

static int ignore_registers = 0;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ivan Baidakou");
MODULE_DESCRIPTION("Microchip MRF89XA SPI Driver");
MODULE_VERSION(MRFSPI_DRV_VERSION);

module_param(ignore_registers, int, S_IRUGO);
MODULE_PARM_DESC(ignore_registers, "Ignore initial register values on probe.");

struct mrf_dev {
  struct cdev cdev;
  struct semaphore semaphore;
  int counter;
  int device_opened;
  struct spi_device *spi;
  struct gpio_desc *config_pin;
  struct gpio_desc *data_pin;
};

struct spi_device* spi_device = NULL;

static int mrf_open(struct inode *inode, struct file *filp) {
  /* int result; */

  /* printk(KERN_INFO "mrf: open device\n"); */

  /* down(&mrf_dev->semaphore); */

  /* if (! mrf_dev->device_opened ) { */
  /*   try_module_get(THIS_MODULE); */
  /*   mrf_dev->device_opened = 1; */
  /*   mrf_dev->counter = 0; */
  /*   result = 0; /\* success *\/ */
  /* } else { */
  /*   result = -EBUSY; */
  /* } */

  /* up(&mrf_dev->semaphore); */
  /* return result; */
  return -EBUSY;
}

static int mrf_release(struct inode *inode, struct file *filp) {
  /* printk(KERN_INFO "mrf: release device\n"); */
  /* down(&mrf_dev->semaphore); */

  /* module_put(THIS_MODULE); */
  /* mrf_dev->device_opened = 0; */

  /* up(&mrf_dev->semaphore); */
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

static struct file_operations mrf_fops = {
 read:    mrf_read,
 write:   mrf_write,
 open:    mrf_open,
 release: mrf_release
};


static u8 default_register_values[] = { 0x28, 0x88, 0x03, 0x07, 0x0C, 0x0F };

static int mrfdev_probe(struct spi_device *spi) {
  u8 i;
  u8 mrf_found = 1;
  int status;
  struct mrf_dev* mrf_dev;

  printk(KERN_INFO "mrf: probing spi device for mrf presence\n");
  mrf_dev = (struct mrf_dev*) spi_get_drvdata(spi);

  gpiod_set_value(mrf_dev->config_pin, 0);
  gpiod_set_value(mrf_dev->data_pin, 0);
  for (i = 0; i < ARRAY_SIZE(default_register_values); i++) {
    u8 got = spi_w8r8(spi, CMD_READ_REGISTER(i));
    u8 expected = default_register_values[i];
    printk(KERN_INFO "mrf: probing %d register. Got: %.2x, expected: %.2x\n", i, got, expected);
    mrf_found &= (got == expected);
  }
  if (mrf_found | ignore_registers) {
    /* success */
    printk(KERN_INFO "mrf: device found\n");
    status = 0;
    spi_device = spi;
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
  int driver_registered = 0;
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

  /* insert new spi device */
  spi = spi_new_device(master, &mrf_board_info);
  if (!spi) {
    printk(KERN_INFO "mrf: cannot add new spi device\n");
    status = -ENODEV;
    goto err;
  }

  /* allocate memory for mrf device */
  mrf_dev = kzalloc(sizeof(struct mrf_dev), GFP_KERNEL);
  if ( !mrf_dev ) {
    status = -ENOMEM;
    goto err;
  }

  /* allocate control and data GPIO pins */
  //mrf_dev->config_pin = gpiod_get(NULL, CSCON_NAME, GPIOD_OUT_LOW);
  mrf_dev->config_pin = gpio_to_desc(CSCON_PIN);
  if (IS_ERR(mrf_dev->config_pin)) {
    printk(KERN_INFO "mrf: cannot get config_pin(%d)\n", CSCON_PIN);
    status = -ENODEV;
    goto err;
  }
  gpiod_direction_output(mrf_dev->config_pin, 0);
  gpiod_set_value(mrf_dev->config_pin, 0);

  mrf_dev->data_pin = gpio_to_desc(DATA_PIN);
  if (IS_ERR(mrf_dev->data_pin)) {
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
  sema_init(&mrf_dev->semaphore, 1);
  cdev_init(&mrf_dev->cdev, &mrf_fops);
  mrf_dev->cdev.owner = THIS_MODULE;
  mrf_dev->cdev.ops = &mrf_fops;
  mrf_dev->spi = spi;

  /* add character device */
  printk(KERN_INFO "mrf: adding character device\n");
  status = cdev_add(&mrf_dev->cdev, device_id, 1);
  if (status) {
    printk(KERN_INFO "mrf: cannot add character deviced\n");
    goto err;
  }
  spi_set_drvdata(spi, mrf_dev);

  /* register and probe mrf device connection via spi */
  status = spi_register_driver(&mrfdev_spi_driver);
  if (status < 0) {
    printk(KERN_INFO "mrf: spi_register_driver failed\n");
    goto err;
  }
  driver_registered = 1;
  /* check that probe succeed */
  if (!spi_device) {
    printk(KERN_INFO "mrf: device hasn't been probed successfully\n");
    status = -ENODEV;
    goto err;
  }
  /* from here spi and spi_device point the same */
  printk(KERN_INFO "mrf: spi device found\n");

  /* all OK */
  printk(KERN_INFO "mrf: initialization succeed\n");
  return 0;

 err:
  printk(KERN_INFO "mrf: failed\n");
  if (driver_registered) spi_unregister_driver(&mrfdev_spi_driver);
  if (device_id) unregister_chrdev_region(device_id, 1);
  if (mrf_dev && mrf_dev->config_pin && !IS_ERR(mrf_dev->config_pin)) gpiod_put(mrf_dev->config_pin);
  if (mrf_dev && mrf_dev->data_pin && IS_ERR(mrf_dev->data_pin)) gpiod_put(mrf_dev->data_pin);
  if (mrf_dev) kfree(mrf_dev);
  if (driver_registered) spi_unregister_driver(&mrfdev_spi_driver);
  if (spi) spi_unregister_device(spi);
  return status;
}

static void __exit mrf_exit(void) {
  struct mrf_dev* mrf_dev;
  dev_t device_id;
  printk(KERN_INFO "mrf: removing module\n");
  /* TODO: check errors */

  mrf_dev = (struct mrf_dev*) spi_get_drvdata(spi_device);
  device_id = mrf_dev->cdev.dev;

  cdev_del(&mrf_dev->cdev);
  printk(KERN_INFO "mrf: character device removed\n");

  unregister_chrdev_region(device_id, 1);
  printk(KERN_INFO "mrf: character device deallocated\n");

  gpiod_put(mrf_dev->config_pin);
  gpiod_put(mrf_dev->data_pin);

  kfree(mrf_dev);
  spi_set_drvdata(spi_device, NULL);
  printk(KERN_INFO "mrf: mrf device memory deallocated\n");

  spi_unregister_driver(&mrfdev_spi_driver);
  printk(KERN_INFO "mrf: spi driver unregistered\n");

  spi_unregister_device(spi_device);
  printk(KERN_INFO "mrf: spi device unregistered\n");
}


module_init(mrf_init);
module_exit(mrf_exit);
