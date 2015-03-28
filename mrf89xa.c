#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/spi/spi.h>

#include <linux/cdev.h>
#include <linux/slab.h> /* kmalloc() */
#include <linux/fs.h> /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/types.h> /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h> /* O_ACCMODE */
#include <asm/uaccess.h> /* copy_from/to_user */
#include <linux/mutex.h>

#define MRFSPI_DRV_NAME "mrfspi"
#define MRFSPI_DRV_VERSION "0.1"
#define MRFSPI_MAX_COUNTER 10
/* raspberry pi has just 0-bus */
#define MRFSPI_BUS_NO 0

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ivan Baidakou");
MODULE_DESCRIPTION("Microchip MRF89XA SPI Driver");
MODULE_VERSION(MRFSPI_DRV_VERSION);

struct mrf_dev {
  struct cdev cdev;
  struct semaphore semaphore;
  int counter;
  int device_opened;
};

static struct mrf_dev* mrf_dev;
static char* device_name = "mrf";
//static dev_t dev = 0;

//static DEFINE_MUTEX(access_mutex);
//static int device_opened = 0;
//static int counter = 0;

static int mrf_open(struct inode *inode, struct file *filp) {
  int result;
  
  printk(KERN_INFO "mrf: open device\n");

  //mutex_lock(&access_mutex);
  down(&mrf_dev->semaphore);

  if (! mrf_dev->device_opened ) {
    try_module_get(THIS_MODULE);
    mrf_dev->device_opened = 1;
    /* success */
    result = 0;
  } else {
    result = -EBUSY;
  }
  
  //mutex_unlock(&access_mutex);
  up(&mrf_dev->semaphore);
  return result;
}

static int mrf_release(struct inode *inode, struct file *filp) {
  printk(KERN_INFO "mrf: release device\n");
  down(&mrf_dev->semaphore);
  
  module_put(THIS_MODULE);
  mrf_dev->device_opened = 0;
  mrf_dev->counter = 0;
  
  up(&mrf_dev->semaphore);
  return 0;
}

static ssize_t mrf_write(struct file *filp, const char *buff, size_t length, loff_t * offset) {
  printk(KERN_INFO "mrf: write to device isn't supported yet\n");
  return -EINVAL;
}

static ssize_t mrf_read(struct file *filp, char *buff, size_t length, loff_t *offset) {
  int bytes_read = 0;
  printk(KERN_INFO "mrf: reading from device\n");

  if (length && mrf_dev->counter < MRFSPI_MAX_COUNTER) {
    put_user('a', buff++);
    length--;
    bytes_read++;
    mrf_dev->counter++;
  }
  return bytes_read;
}

static struct file_operations mrf_fops = {
 read:    mrf_read,
 write:   mrf_write,
 open:    mrf_open,
 release: mrf_release
};

static int spi_device_found(struct device *dev, void *data)
{
  struct spi_device *spi = container_of(dev, struct spi_device, dev);

  printk(KERN_INFO "found spi device %s %s %dkHz %d bits mode=0x%02X\n",
         spi->modalias, dev_name(dev), spi->max_speed_hz/1000,
         spi->bits_per_word, spi->mode);

  printk(KERN_INFO "bus no :%d\n", spi->master->bus_num);
        
  return 0;
}


static void pr_spi_devices(void)
{
  printk(KERN_INFO ":  SPI devices registered:\n");
  bus_for_each_dev(&spi_bus_type, NULL, NULL, spi_device_found);
}

static __init int mrf_init(void) {
  int result;
  int dev_no;
  dev_t dev = 0;
  struct spi_master *master;
  int device_major;

  printk(KERN_INFO "mrf_init\n");
  pr_spi_devices();
  // drivers/staging/fbtft/fbtft_device.c
  /*
  master = spi_busnum_to_master(MRFSPI_BUS_NO);
  printk(KERN_INFO " master spi found : %x \n", master);
  */
  
  /* dynamically allocatin major number */
  /*
  result =  alloc_chrdev_region(&dev, 0, 1, device_name);
  if (result) {
    printk(KERN_ERR "mrf: failed to register chrdev region\n");
    goto err;
  }

  printk(KERN_INFO "mrf: allocated major device numer: %d\n", MAJOR(dev));
  */
  /* registering character device */

  /*
  result = register_chrdev(0, device_name, &mrf_fops);
  if (result < 0) { 
    printk(KERN_ERR "mrf: failed to register character device\n");
    goto err;
  }
  printk(KERN_INFO "mrf: allocated major device numer: %d\n", result);
  */

  result = alloc_chrdev_region(&dev, 1, 1, device_name);
  if ( result < 0 ) {
    goto err;
  }
  device_major = MAJOR(dev);
  printk(KERN_INFO "mrf: allocated major device numer: %d\n", device_major);
  
  mrf_dev = kmalloc(sizeof(struct mrf_dev), GFP_KERNEL);
  if ( !mrf_dev ) {
    result = -ENOMEM;
    goto err;
  }
  memset(mrf_dev, sizeof(struct mrf_dev), 0);
  sema_init(&mrf_dev->semaphore, 1);
  cdev_init(&mrf_dev->cdev, &mrf_fops);
  mrf_dev->cdev.owner = THIS_MODULE;
  mrf_dev->cdev.ops = &mrf_fops;
  result = cdev_add (&mrf_dev->cdev, dev, 1);
  if (result) {
    goto err;
  }
  
  /* all OK */
  printk(KERN_INFO "mrf: initialization succeed\n");
  return 0;

 err:
  if (mrf_dev) kfree(mrf_dev);
  return result;
}

static void __exit mrf_exit(void) {
  /*
  unregister_chrdev(MAJOR(dev), device_name);
  */
  
  printk(KERN_INFO "mrf: removing module\n");
  cdev_del(&mrf_dev->cdev);
  kfree(mrf_dev);
}


module_init(mrf_init);
module_exit(mrf_exit);
