#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#define GPIO_NUMBER		60     	// 60 = gpio1_28 (1 * 32 + 28 = 60) // P9_12 on Beaglebone Black
#define GPIO_SECOND		30		// 30 = gpio0_30 (0 * 32 + 30 = 30) // P9_11 on Beaglebone Black
#define DEBOUNCE_DELAY 500

static dev_t first; // Global variable for the first device number
static struct cdev c_dev, c_dev2; // Global variable for the character device structure
static struct class *cl; // Global variable for the device class

static int init_result;
static int button1_irq = 0;
static int button2_irq = 0;
static int button1_count = 0; // irq count
static int button2_count = 0; // irq count

static int buttons = 1;
module_param(buttons, int, 0);
MODULE_PARM_DESC(buttons, "This byte decides the number of buttons (1-2)");

unsigned int last_interrupt_time1 = 0;
unsigned int last_interrupt_time2 = 0;
static uint64_t epochMilli;

/*
 * Timer for interrupt debounce, borrowed from
 * http://raspberrypi.stackexchange.com/questions/8544/gpio-interrupt-debounce
 */
unsigned int millis(void) {
	struct timeval tv;
	uint64_t now;

	do_gettimeofday(&tv);
	now = (uint64_t) tv.tv_sec * (uint64_t) 1000
			+ (uint64_t)(tv.tv_usec / 1000);

	return (uint32_t)(now - epochMilli);
}

/*
 * The interrupt handler function
 */
static irqreturn_t button1_handler(int irq, void *dev_id) {
	unsigned int interrupt_time = millis();

	if (interrupt_time - last_interrupt_time1 < DEBOUNCE_DELAY) {
		//printk(KERN_NOTICE "button: Ignored Interrupt! \n");
		return IRQ_HANDLED;
	}
	last_interrupt_time1 = interrupt_time;

	button1_count++;
	printk(KERN_INFO "button: I got an interrupt.\n");
	return IRQ_HANDLED;
}
static irqreturn_t button2_handler(int irq, void *dev_id) {
	unsigned int interrupt_time = millis();

	if (interrupt_time - last_interrupt_time2 < DEBOUNCE_DELAY) {
		//printk(KERN_NOTICE "button: Ignored Interrupt! \n");
		return IRQ_HANDLED;
	}
	last_interrupt_time2 = interrupt_time;

	button2_count++;
	printk(KERN_INFO "button: I got an interrupt.\n");
	return IRQ_HANDLED;
}

/*
 * .read
 */
static ssize_t button1_read(struct file* F, char *buf, size_t count,
		loff_t *f_pos) {
	printk(KERN_INFO "button irq counts: %d\n", button1_count);

	char buffer[10];

	int temp = gpio_get_value(GPIO_NUMBER);

	sprintf(buffer, "%1d", temp);

	count = sizeof(buffer);

	if (copy_to_user(buf, buffer, count)) {
		return -EFAULT;
	}

	if (*f_pos == 0) {
		*f_pos += 1;
		return 1;
	} else {
		return 0;
	}
}

static ssize_t button2_read(struct file* F, char *buf, size_t count,
		loff_t *f_pos) {
	printk(KERN_INFO "button irq counts: %d\n", button2_count);

	char buffer[10];

	int temp = gpio_get_value(GPIO_SECOND);

	sprintf(buffer, "%1d", temp);

	count = sizeof(buffer);

	if (copy_to_user(buf, buffer, count)) {
		return -EFAULT;
	}

	if (*f_pos == 0) {
		*f_pos += 1;
		return 1;
	} else {
		return 0;
	}
}

/*
 * .write
 */
static ssize_t button1_write(struct file* F, const char *buf, size_t count,
		loff_t *f_pos) {
	printk(KERN_INFO "button: Executing WRITE.\n");

	switch (buf[0]) {
	case '0':
		gpio_set_value(GPIO_NUMBER, 0);
		button1_count = 0;
		break;
	case '1':
		gpio_set_value(GPIO_NUMBER, 1);
		break;

	default:
		printk("button: Wrong option.\n");
		break;
	}

	return count;
}
static ssize_t button2_write(struct file* F, const char *buf, size_t count,
		loff_t *f_pos) {
	printk(KERN_INFO "button: Executing WRITE.\n");

	switch (buf[0]) {
	case '0':
		gpio_set_value(GPIO_SECOND, 0);
		button2_count = 0;
		break;
	case '1':
		gpio_set_value(GPIO_SECOND, 1);
		break;

	default:
		printk("button: Wrong option.\n");
		break;
	}

	return count;
}

/*
 * / .open
 */
static int button_open(struct inode *inode, struct file *file) {
	return 0;
}

/*
 * .close
 */
static int button_close(struct inode *inode, struct file *file) {
	return 0;
}

static struct file_operations FileOps1 = { .owner = THIS_MODULE, .open =
		button_open, .read = button1_read, .write = button1_write, .release =
		button_close, };

static struct file_operations FileOps2 = { .owner = THIS_MODULE, .open =
		button_open, .read = button2_read, .write = button2_write, .release =
		button_close, };
/*
 * Module init function.
 */
static int __init init_button(void)
{
	printk(KERN_ALERT "buttons value is: %i\n", buttons);

	struct timeval tv;
	//init_result = register_chrdev( 0, "gpio", &FileOps );

	do_gettimeofday(&tv);
	epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000);

	init_result = alloc_chrdev_region( &first, 0, 2, "button_driver" );

	if( 0 > init_result )
	{
		printk( KERN_ALERT "button: Device Registration failed\n" );
		return -1;
	}
	else
	{
		printk( KERN_ALERT "button: Major number is: %d\n",init_result );
		//return 0;
	}

	if ( (cl = class_create( THIS_MODULE, "chardev" ) ) == NULL )
	{
		printk( KERN_ALERT "button: Class creation failed\n" );
		unregister_chrdev_region( first, 1 );
		return -1;
	}
	/*
	 * First device
	 */
	if(buttons >= 1) {
		if( device_create( cl, NULL, first, NULL, "button1" ) == NULL )
		{
			printk( KERN_ALERT "button: Device creation failed\n" );
			class_destroy(cl);
			unregister_chrdev_region( first, 2 );
			return -1;
		}
		cdev_init( &c_dev, &FileOps1 );

		if( cdev_add( &c_dev, first, 1 ) == -1)
		{
			printk( KERN_ALERT "Device addition failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 2 );
			return -1;
		}

		if(gpio_request(GPIO_NUMBER, "button1"))
		{
			printk( KERN_ALERT "gpio request failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 2 );
			return -1;
		}

		if((button1_irq = gpio_to_irq(GPIO_NUMBER)) < 0)
		{
			printk( KERN_ALERT "gpio to irq failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 2 );
			return -1;
		}

		if(request_irq(button1_irq, button1_handler, IRQF_TRIGGER_RISING | IRQF_DISABLED, "gpiomod#button", NULL ) == -1)
		{
			printk( KERN_ALERT "Device interrupt handle failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 1 );

			return -1;
		}
		else
		{
			printk( KERN_ALERT "button: Device irq number is %d\n", button1_irq );
		}
	}

	/*
	 * Second Device
	 */
	if(buttons >= 2) {
		if( device_create( cl, NULL, first + 1, NULL, "button2" ) == NULL )
		{
			printk( KERN_ALERT "button: Device creation failed\n" );
			class_destroy(cl);
			unregister_chrdev_region( first, 2 );
			return -1;
		}

		cdev_init( &c_dev2, &FileOps2 );

		if( cdev_add( &c_dev2, first + 1, 1 ) == -1)
		{
			printk( KERN_ALERT "Device addition failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 2 );
			return -1;
		}

		if(gpio_request(GPIO_SECOND, "button1"))
		{
			printk( KERN_ALERT "gpio request failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 2 );
			return -1;
		}

		if((button2_irq = gpio_to_irq(GPIO_SECOND)) < 0)
		{
			printk( KERN_ALERT "gpio to irq failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 2 );
			return -1;
		}
		if(request_irq(button2_irq, button2_handler, IRQF_TRIGGER_RISING | IRQF_DISABLED, "gpiomod#button", NULL ) == -1)
		{
			printk( KERN_ALERT "Device interrupt handle failed\n" );
			device_destroy( cl, first );
			class_destroy( cl );
			unregister_chrdev_region( first, 1 );

			return -1;
		}
		else
		{
			printk( KERN_ALERT "button: Device irq number is %d\n", button2_irq );
		}
	}

	return 0;
}
/*
 * Module exit function
 */
static void __exit cleanup_button(void)
{
	cdev_del( &c_dev );
	device_destroy( cl, first );
	device_destroy( cl, first + 1 );
	class_destroy( cl );
	unregister_chrdev_region( first, 2 );

	printk(KERN_ALERT "button: Device unregistered\n");

	free_irq(button1_irq, NULL);
	free_irq(button2_irq, NULL);
	gpio_free(GPIO_NUMBER);
	gpio_free(GPIO_SECOND);
}

module_init( init_button);
module_exit( cleanup_button);

MODULE_AUTHOR("Birkir Oskarsson & Bjorn Smith");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Beaglebone Black gpio1_28 irq driver");
