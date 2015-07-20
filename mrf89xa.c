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
#include <asm/byteorder.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/interrupt.h>
#include <linux/timer.h>

#include "mrf89xa.h"

#define MRFSPI_DRV_NAME "mrfspi"
#define MRFSPI_DRV_VERSION "0.1"

long mrf_ioctl_unlocked(struct file *filp, unsigned int cmd, unsigned long arg);
static int write_register(u8 index, u8 value);
static u8 read_register(u8 index);
static irqreturn_t irq0_handler(int, void *);
static irqreturn_t irq1_handler(int, void *);
static int set_chip_mode(u8 mode);
static int write_fifo(u8 address, u8 length, u8* data);
static void tx_timeout(unsigned long);
static int _tx_queue_size(void);

static int ignore_registers = 0;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ivan Baidakou");
MODULE_DESCRIPTION("Microchip MRF89XA SPI Driver");
MODULE_VERSION(MRFSPI_DRV_VERSION);

module_param(ignore_registers, int, S_IRUGO);
MODULE_PARM_DESC(ignore_registers, "Ignore initial register values on probe.");


struct mrf_dev {
  struct cdev cdev;
  spinlock_t state_lock;
  uint32_t state;

  int irq0;
  int irq1;

  struct timer_list tx_timeout_timer;
  /* protects access to driver */
  struct semaphore driver_semaphore;
  struct spi_device *spi;
  struct gpio_desc *config_pin;
  struct gpio_desc *data_pin;
  struct gpio_desc *reset_pin;
  struct proc_dir_entry* proc_file;

  spinlock_t tx_queue_lock;
  u8 tx_queue_size;
  struct list_head tx_queue;
};

/* mimics mrf_frame, but embeds size */
struct mrf_payload {
  u8 dest;
  u8 size;
  u8 data[PAYLOAD_64];
  struct list_head list_item;
};

struct mrf_dev *mrf_device = NULL;
static const char* mrf_device_name = MRFSPI_DRV_NAME;

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
  int state;
  struct gpio_desc *irq1_desc;

  printk(KERN_INFO "mrf: open device %p\n", mrf_device);

  down(&mrf_device->driver_semaphore);

  spin_lock(&mrf_device->state_lock);
  state = mrf_device->state;
  spin_unlock(&mrf_device->state_lock);


  if (! (state & MRF_STATE_DEVICEOPENED) ) {
    int irq0, irq1;

    irq0 = gpio_to_irq(IRQ0_PIN);
    if (irq0 < 0) {
      status = irq0;
      printk(KERN_WARNING "mrf: cannot get irq0 via %d pin (status: %d)\n", IRQ0_PIN, status);
      goto finish;
    }
    status = request_irq(irq0, &irq0_handler, 0, mrf_device_name, mrf_device);
    if (status) goto finish;
    mrf_device->irq0 = irq0;
    printk(KERN_INFO "mrf: irq0 %d\n", irq0);


    irq1_desc = gpio_to_desc(IRQ1_PIN);
    if (IS_ERR_OR_NULL(irq1_desc)) {
      printk(KERN_INFO "mrf: irq1 pin (%d) not found", IRQ1_PIN);
      goto finish;
    }
    status = gpiod_direction_input(irq1_desc);
    if (status) {
      printk(KERN_INFO "mrf: irq1 pin (%d) error setting input direction", IRQ1_PIN);
      goto finish;
    }
    irq1 = gpiod_to_irq(irq1_desc);
    if (irq1 < 0) {
      status = irq1;
      printk(KERN_WARNING "mrf: cannot get irq1 via %d pin (status: %d)\n", IRQ1_PIN, status);
      goto finish;
    }

    status = request_irq(irq1, &irq1_handler,
                         IRQF_TRIGGER_RISING,
                         mrf_device_name, mrf_device);
    if (status) goto finish;
    mrf_device->irq1 = irq1;
    printk(KERN_INFO "mrf: irq1 %d\n", irq1);

    try_module_get(THIS_MODULE);

    spin_lock(&mrf_device->state_lock);
    mrf_device->state |= MRF_STATE_DEVICEOPENED;
    spin_unlock(&mrf_device->state_lock);

    status = 0; /* success */
  } else {
    status = -EBUSY;
  }

 finish:
  up(&mrf_device->driver_semaphore);
  if (status) { /* error */
    if (mrf_device->irq0) free_irq(mrf_device->irq0, mrf_device);
    if (mrf_device->irq1) free_irq(mrf_device->irq1, mrf_device);
    module_put(THIS_MODULE);
  }

  return status;
}

static int mrf_release(struct inode *inode, struct file *filp) {
  printk(KERN_INFO "mrf: release device\n");
  down(&mrf_device->driver_semaphore);

  free_irq(mrf_device->irq0, mrf_device);
  free_irq(mrf_device->irq1, mrf_device);

  module_put(THIS_MODULE);

  spin_lock(&mrf_device->state_lock);
  mrf_device->state &= ~(MRF_STATE_DEVICEOPENED);
  spin_unlock(&mrf_device->state_lock);

  up(&mrf_device->driver_semaphore);
  return 0;
}

static ssize_t mrf_write(struct file *filp, const char *buff, size_t length, loff_t * offset) {
  int status;
  int data_size;
  u8 dest;
  struct mrf_payload *payload = NULL;

  down(&mrf_device->driver_semaphore);

  /* queue size could extrenally be decreased only */
  if ((filp->f_flags & O_NONBLOCK) && (_tx_queue_size() >= MRF_MAX_TX_QUEUE)) {
    status = -EAGAIN;
    goto finish;
  }

  /* check input data */
  if (offset) { status = -EINVAL; goto finish; }
  data_size = length - sizeof(((mrf_frame*)0)->dest);
  if (data_size <= 0) {
    /* nothing to send? */
    status = -EINVAL;
    goto finish;
  }
  if (!access_ok(VERIFY_READ, buff, length)) {
    status = -EFAULT;
    goto finish;
  }
  payload = kzalloc(sizeof(struct mrf_payload), GFP_KERNEL);
  if ( !payload ) {
    status = -ENOMEM;
    goto finish;
  }

  status = __get_user(dest, (u8*)buff);
  if ( !status) { goto finish; }
  payload->dest = dest;

  if (copy_from_user(&payload->data, buff + sizeof(u8), data_size)) {
     status = -EFAULT;
     goto finish;
  }
  payload->size = data_size;

  /* check complete */

  spin_lock(&mrf_device->tx_queue_lock);
  list_add_tail(&payload->list_item, &mrf_device->tx_queue);
  mrf_device->tx_queue_size++;
  spin_unlock(&mrf_device->tx_queue_lock);

 finish:
  up(&mrf_device->driver_semaphore);
  if (status <0 && payload) kfree(payload);
  return status;
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

static int _tx_queue_size(void) {
  int result;
  spin_lock(&mrf_device->tx_queue_lock);
  result = mrf_device->tx_queue_size;
  spin_unlock(&mrf_device->tx_queue_lock);
  return result;
}


static int mrf_dump_stats(struct seq_file *m, void *v){
  int i, j;
  u8 regs[32];
  u8 mode_id;
  u32 network_id;
  u64 freq;
  const char* mode;
  struct gpio_desc *irq_pins[2];

  down(&mrf_device->driver_semaphore);

  /* print all 32 registers, 4 per row */
  for (i = 0; i < 32; i++) {
    regs[i] = read_register(i);
  }

  seq_printf(m, "mrf dump\n");
  network_id = (regs[REG_SYNC_WORD_1] << 8*3) | (regs[REG_SYNC_WORD_2] << 8*2)
             | (regs[REG_SYNC_WORD_3] << 8*1) | (regs[REG_SYNC_WORD_4] << 8*0);
  mode_id = regs[REG_GCON] & CHIPMODE_MASK;
  mode = (mode_id  == CHIPMODE_TX)
    ? "tx"
    : (mode_id == CHIPMODE_RX)
    ? "rx"
    : (mode_id == CHIPMODE_FSMODE)
    ? "fs"
    : (mode_id == CHIPMODE_STBYMODE)
    ? "stand-by"
    : "sleep"
    ;

  seq_printf(m, "mode: %s\n", mode);
  seq_printf(m, "node address: 0x%.2x, network: 0x%.8x\n", regs[REG_NADDS], network_id);

  freq = ((MRF_CRYSTALL_FREQ)/((regs[REG_R1C]+1) * 8)) * ((75*(regs[REG_P1C] + 1) + regs[REG_S1C]) * 9);
  seq_printf(m, "frequency: %llu Hz\n", freq);

  seq_printf(m, "raw register values\n");
  for (i = 0; i < 8; i++){
    for (j = 0; j < 4; j++) {
      int index = i*4 + j;
      u8 value = regs[index];
      seq_printf(m, "register %02d (0x%.2x) = 0x%.2x %s", index + 1, index, value, (j == 3) ? "\n" : "| ");
    }
  }

  irq_pins[0] = gpio_to_desc(IRQ0_PIN);
  if (IS_ERR_OR_NULL(irq_pins[0])) {
    printk(KERN_INFO "mrf: irq0 pin (%d) not found", IRQ0_PIN);
    goto finish;
  }
  gpiod_direction_input(irq_pins[0]);

  irq_pins[1] = gpio_to_desc(IRQ1_PIN);
  if (IS_ERR_OR_NULL(irq_pins[1])) {
    printk(KERN_INFO "mrf: irq1 pin (%d) not found", IRQ1_PIN);
    goto finish;
  }
  gpiod_direction_input(irq_pins[1]);

  seq_printf(m, "pin %d value = %d, pin %d value = %d\n",
             IRQ0_PIN, gpiod_get_value(irq_pins[0]),
             IRQ1_PIN, gpiod_get_value(irq_pins[1]) );

 finish:
  up(&mrf_device->driver_semaphore);
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
  int status;
  gpiod_set_value(mrf_device->config_pin, 0);
  status = spi_write(mrf_device->spi, buff, ARRAY_SIZE(buff));
  gpiod_set_value(mrf_device->config_pin, 1);

  return status;
}

static int write_fifo(u8 address, u8 length, u8* data) {
  int status;
  u8 i;
  /* add address byte to the length of fifo buffer */
  u8 total_lenght = length + 1;

  struct spi_transfer t = {
    .tx_buf		= &total_lenght,
    /* the same for all transfers */
    .len		= 1,
    /* .speed_hz   = 500000, */
  };

  /* trashfer length */
  gpiod_set_value(mrf_device->data_pin, 0);
  status = spi_sync_transfer(mrf_device->spi, &t, 1);
  gpiod_set_value(mrf_device->data_pin, 1);
  if (status) goto finish;

  /* trashfer address */
  t.tx_buf = &address;
  gpiod_set_value(mrf_device->data_pin, 0);
  status = spi_sync_transfer(mrf_device->spi, &t, 1);
  gpiod_set_value(mrf_device->data_pin, 1);
  if (status) goto finish;

  /* by-byte transfer data */
  for (i = 0; i < length; i++) {
    t.tx_buf = data + i;
    gpiod_set_value(mrf_device->data_pin, 0);
    status = spi_sync_transfer(mrf_device->spi, &t, 1);
    gpiod_set_value(mrf_device->data_pin, 1);
    if (status) goto finish;
  }

 finish:
  return status;
}

static u8 read_register(u8 index) {
  u8 value;
  gpiod_set_value(mrf_device->config_pin, 0);
  value = spi_w8r8(mrf_device->spi, CMD_READ_REGISTER(index));
  gpiod_set_value(mrf_device->config_pin, 1);
  return value;
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

static void tx_timeout(unsigned long unused) {
  printk(KERN_INFO "mrf: tx_timeout\n");

  spin_lock(&mrf_device->state_lock);
  mrf_device->state &= ~(MRF_STATE_TRANSMITTING);
  spin_unlock(&mrf_device->state_lock);
}


static irqreturn_t irq0_handler(int a, void *b) {
  printk(KERN_INFO "mrf: irq0_handler\n");
  return IRQ_HANDLED;
}

static irqreturn_t irq1_handler(int a, void *b) {

  spin_lock(&mrf_device->state_lock);
  if ( mrf_device->state & MRF_STATE_TRANSMITTING ) {
    mrf_device->state &= ~(MRF_STATE_TRANSMITTING);
    del_timer(&mrf_device->tx_timeout_timer);
    printk(KERN_INFO "mrf: irq1_handler / timer removed\n");
  } else {
    printk(KERN_WARNING "mrf: irq1_handler is called with unknown state : 0x%X, ignoring ?\n",
           mrf_device->state);
  }
  spin_unlock(&mrf_device->state_lock);

  return IRQ_HANDLED;
}

static int set_chip_mode(u8 mode) {
  u8 value;

  value = read_register(REG_GCON) & (~CHIPMODE_MASK);
  value |= mode;
  return write_register(REG_GCON, value);
}

static int transfer_data(u8 address, u8 length, u8* data) {
  int status;
  u8 access = read_register(REG_FCRC) & (~FIFO_STBY_ACCESS_MASK);
  printk(KERN_INFO "mrf: tmp set chip mode to sleep\n");

  if ((status = set_chip_mode(CHIPMODE_STBYMODE))) goto finish;

  /* switch to write fifo mode */
  access |= FIFO_STBY_ACCESS_WRITE;
  if ((status = write_register(REG_FCRC, access))) {
    goto finish;
  }
  /* lock current frequency */
  if ((status = write_register(REG_FTPRI, default_register_values[REG_FTPRI] | IRQ1_PLL_LOCK))){
    goto finish;
  }
  /* clear FIFO and FIFO overrun flag */
  if ((status = write_register(REG_FTXRXI, default_register_values[REG_FTXRXI] | IRQ1_FIFO_OVERRUN_CLEAR))) {
    goto finish;
  }
  /* send to broadcast address some dummy data*/
  if ((status = write_fifo(0x00, length, data))) {
    goto finish;
  }
  if ((status = set_chip_mode(CHIPMODE_TX))) {
    goto finish;
  }
  /* tx timeout timer */
  init_timer(&mrf_device->tx_timeout_timer);

  mrf_device->tx_timeout_timer.function = tx_timeout;
  mrf_device->tx_timeout_timer.expires = jiffies + MRF_IRQ_TX_TIMEOUT * HZ / 1000;

  spin_lock(&mrf_device->state_lock);
  mrf_device->state |= MRF_STATE_TRANSMITTING;
  spin_unlock(&mrf_device->state_lock);

  /* start tx timeout timer */
  add_timer(&mrf_device->tx_timeout_timer);

  printk(KERN_INFO "mrf: tmp success\n");

 finish:
  return status;
}

static int reset_mrf(int reinitialize) {
  int status;

  down(&mrf_device->driver_semaphore);

  /* disable mrf interrupts during reset */
  spin_lock(&mrf_device->state_lock);
  mrf_device->state &= ~(MRF_STATE_ADDRESSASSIGNED);
  mrf_device->state &= ~(MRF_STATE_FREQASSIGNED);
  mrf_device->state &= ~(MRF_STATE_TRANSMITTING);
  spin_unlock(&mrf_device->state_lock);

  disable_irq(mrf_device->irq0);
  disable_irq(mrf_device->irq1);

  gpiod_set_value(mrf_device->reset_pin, 1);

  /* signal reset */
  msleep(RESET_DELAY);

  gpiod_set_value(mrf_device->reset_pin, 0);
  /* wait until device will be up */
  msleep(RESET_DELAY);

  if (reinitialize) {
    status = initialize_registers();
    if (status) goto finish;
  }
  printk(KERN_INFO "mrf: device reset successfull\n");

 finish:
  /* restore interrupt handlers */
  enable_irq(mrf_device->irq0);
  enable_irq(mrf_device->irq1);

  up(&mrf_device->driver_semaphore);
  return status;
}

long mrf_ioctl_unlocked(struct file *filp, unsigned int cmd, unsigned long arg) {
  int status = 0;
#define write_register_protected(ADDR, VALUE) status = write_register((ADDR), (VALUE)); if (status) goto finish;

  printk(KERN_INFO "mrf: ioctl (%d)\n", cmd);

  if (_IOC_TYPE(cmd) != MRF_IOC_MAGIC) return -ENOTTY;
  if (_IOC_NR(cmd) > MRF_IOC_MAXNR) return -ENOTTY;

  if (_IOC_DIR(cmd) & _IOC_READ)
    status = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
  else if (_IOC_DIR(cmd) & _IOC_WRITE)
    status = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
  if (status) return -EFAULT;

  printk(KERN_INFO "mrf: ioctl switch\n");
  switch(cmd) {
  case MRF_IOC_RESET:
    reset_mrf(arg);
    break;
  case MRF_IOC_SETADDR:
    {
      mrf_address *addr = (mrf_address*) arg;
      u32 network_id = cpu_to_be32(addr->network_id);
      uint8_t *network_parts = (uint8_t*) &network_id;
      if (addr->node_id == MRF_BROADCAST_NODEADDR ) {
        status = -ENOTTY;
        goto finish;
      }
      printk(KERN_INFO "mrf: set node address: 0x%.2x\n", addr->node_id);
      write_register_protected(REG_NADDS, addr->node_id);
      write_register_protected(REG_SYNC_WORD_1, network_parts[0]);
      write_register_protected(REG_SYNC_WORD_2, network_parts[1]);
      write_register_protected(REG_SYNC_WORD_3, network_parts[2]);
      write_register_protected(REG_SYNC_WORD_4, network_parts[3]);

      spin_lock(&mrf_device->state_lock);
      mrf_device->state |= MRF_STATE_ADDRESSASSIGNED;
      spin_unlock(&mrf_device->state_lock);
    }
    break;
  case MRF_IOC_SETFREQ:
    {
      uint32_t value = cpu_to_be32(arg);
      uint8_t *rps = (uint8_t*) &value;
      write_register_protected(REG_R1C, *rps++);
      write_register_protected(REG_P1C, *rps++);
      write_register_protected(REG_S1C, *rps++);

      spin_lock(&mrf_device->state_lock);
      mrf_device->state |= MRF_STATE_FREQASSIGNED;
      spin_unlock(&mrf_device->state_lock);
    }
    break;
  case MRF_IOC_SETPOWER:
    {
      uint8_t power_level = (uint8_t) arg;
      printk(KERN_INFO "mrf: set power to: 0x%.2x\n", power_level);
      write_register_protected(REG_TXCON, (FC_400 | power_level));
    }
    break;
  case MRF_IOC_DEBUG:
    {
      u8 data[5] = {0x1, 0x2, 0x3, 0x4, 0xFF};
      u8 dst = 0x0; /* broadcast address */
      transfer_data(dst, ARRAY_SIZE(data), data);
    }
    break;
  default:  /* redundant, as cmd was checked against MAXNR */
    return -ENOTTY;
  };

 finish:
  printk(KERN_INFO "mrf: ioctl result: %d\n", status);
  return status;
}


static int mrfdev_probe(struct spi_device *spi) {
  u8 i;
  u8 mrf_found = 1;
  int status;

  printk(KERN_INFO "mrf: probing spi device %p for mrf presence\n", spi);

  for (i = 0; i < ARRAY_SIZE(por_register_values); i++) {
    u8 got;
    u8 expected = por_register_values[i];

    gpiod_set_value(mrf_device->config_pin, 0);
    got = spi_w8r8(spi, CMD_READ_REGISTER(i));
    gpiod_set_value(mrf_device->config_pin, 1);

    printk(KERN_INFO "mrf: probing %d register. Got: %.2x, expected: %.2x\n", i, got, expected);
    mrf_found &= (got == expected);
  }
  if (mrf_found | ignore_registers) {
    /* success */
    printk(KERN_INFO "mrf: device found\n");
    status = 0;

    /* no need to lock, invoked only once from init */
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
  //.max_speed_hz = 100000, /* 500 000, 1800000 */
  .chip_select = 0,
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
  gpiod_set_value(mrf_dev->config_pin, 1);

  mrf_dev->data_pin = gpio_to_desc(DATA_PIN);
  if (IS_ERR_OR_NULL(mrf_dev->data_pin)) {
    printk(KERN_INFO "mrf: cannot get data_pin(%d)\n", DATA_PIN);
    status = -ENODEV;
    goto err;
  }
  gpiod_direction_output(mrf_dev->data_pin, 0);
  gpiod_set_value(mrf_dev->data_pin, 1);

  mrf_dev->reset_pin = gpio_to_desc(RESET_PIN);
  if (IS_ERR_OR_NULL(mrf_dev->reset_pin)) {
    printk(KERN_INFO "mrf: reset pin (%d) not found", RESET_PIN);
    status = -EFAULT;
    goto err;
  }
  gpiod_direction_output(mrf_dev->reset_pin, 0);
  gpiod_set_value(mrf_dev->reset_pin, 0);

  /* characted device allocation */
  status = alloc_chrdev_region(&device_id, 1, 1, "mrf");
  if ( status < 0 ) {
    printk(KERN_INFO "mrf: cannot allocate character device\n");
    goto err;
  }
  printk(KERN_INFO "mrf: allocated major device numer: %d\n", MAJOR(device_id));

  /* initialize mrf device */
  INIT_LIST_HEAD(&mrf_dev->tx_queue);
  mrf_dev->tx_queue_size = 0;
  spin_lock_init(&mrf_dev->tx_queue_lock);

  sema_init(&mrf_dev->driver_semaphore, 1);
  cdev_init(&mrf_dev->cdev, &mrf_fops);
  mrf_dev->cdev.owner = THIS_MODULE;
  mrf_dev->cdev.ops = &mrf_fops;
  mrf_dev->spi = spi;
  /* no need to lock */
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
  /* check that probe succeed; no need to lock */
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
  if (mrf_dev && mrf_dev->reset_pin && !IS_ERR_OR_NULL(mrf_dev->reset_pin)) gpiod_put(mrf_dev->reset_pin);
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
  gpiod_put(mrf_device->reset_pin);

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
