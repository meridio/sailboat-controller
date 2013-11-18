/*
 * hall_new.c
 *
 *  Created on: Nov 18, 2013
 *      Author: Bjørn Smith @ SDU.dk
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/types.h>

#define GPIO_HALLA		60     	// 60 = gpio1_28 (1 * 32 + 28 = 60) // P9_12 on Beaglebone Black
#define GPIO_HALLB		30		// 30 = gpio0_30 (0 * 32 + 30 = 30) // P9_11 on Beaglebone Black
#define DEBOUNCE_DELAY 	50
#define HALL_MAX		500		//The length of sail actuator!
static dev_t first; // Global variable for the first device number
static struct cdev c_dev; //, c_dev2; // Global variable for the character device structure
static struct class *cl; // Global variable for the device class

static int init_result;
static int hall_a_irq = 0;
static int hall_b_irq = 0;
static int hall_length = 0; // irq count
static int hall_state = 0;

unsigned int last_interrupt_time = 0;
static uint64_t epochMilli;

/*************IRQ state flow**********************
 * A:____|-----|____|-----|____|-----|__
 * B:______|-----|____|-----|____|-----|
 *
 * S:    0 1   2 3  0 1   2 3
 *
 * Concept: Check state before and after to deside direction of movement.
 *
 * Interrupts needed for both falling and rising edges
 * State | HallA  | HallB  |
 * S0    | rising | low    |
 * S1    | high   | rising |
 * S2    | falling| high   |
 * S3    | low    | falling|
 *
 *
 */

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
 * The interrupt handler functions
 */
static irqreturn_t halla_rising_handler(int irq, void *dev_id) {
	unsigned int interrupt_time = millis();

	if (interrupt_time - last_interrupt_time < DEBOUNCE_DELAY) {
		//printk(KERN_NOTICE "button: Ignored Interrupt! \n");
		return IRQ_HANDLED;
	}
	last_interrupt_time = interrupt_time;
	printk(KERN_INFO "halla: I got rising interrupt.\n");

	if (hall_state == 1) {
		hall_length--;
	} else if (hall_state == 3) {
		hall_length++;
	}
	hall_state = 0;

	return IRQ_HANDLED;
}

static irqreturn_t halla_falling_handler(int irq, void *dev_id) {
	unsigned int interrupt_time = millis();

	if (interrupt_time - last_interrupt_time < DEBOUNCE_DELAY) {
		//printk(KERN_NOTICE "button: Ignored Interrupt! \n");
		return IRQ_HANDLED;
	}
	last_interrupt_time = interrupt_time;

	printk(KERN_INFO "halla: I got falling interrupt.\n");

	if (hall_state == 1) {
		hall_length++;
	} else if (hall_state == 3) {
		hall_length--;
	}
	hall_state = 2;

	return IRQ_HANDLED;
}
static irqreturn_t hallb_rising_handler(int irq, void *dev_id) {
	unsigned int interrupt_time = millis();

	if (interrupt_time - last_interrupt_time < DEBOUNCE_DELAY) {
		//printk(KERN_NOTICE "button: Ignored Interrupt! \n");
		return IRQ_HANDLED;
	}
	last_interrupt_time = interrupt_time;

	printk(KERN_INFO "hallb: I got rising interrupt.\n");

	if (hall_state == 0) {
		hall_length++;
	} else if (hall_state == 2) {
		hall_length--;
	}
	hall_state = 1;

	return IRQ_HANDLED;
}

static irqreturn_t hallb_falling_handler(int irq, void *dev_id) {
	unsigned int interrupt_time = millis();

	if (interrupt_time - last_interrupt_time < DEBOUNCE_DELAY) {
		//printk(KERN_NOTICE "button: Ignored Interrupt! \n");
		return IRQ_HANDLED;
	}
	last_interrupt_time = interrupt_time;

	printk(KERN_INFO "hallb: I got falling interrupt.\n");

	if (hall_state == 2) {
		hall_length++;
	} else if (hall_state == 0) {
		hall_length--;
	}
	hall_state = 3;

	return IRQ_HANDLED;
}

/*
 * .read
 */
static ssize_t hall_read(struct file* F, char *buf, size_t count, loff_t *f_pos) {
	printk(KERN_INFO "hall length: %d\n", hall_count);

	char buffer[10];

	int temp = hall_length;

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
static ssize_t hall_write(struct file* F, const char *buf, size_t count,
		loff_t *f_pos) {
	printk(KERN_INFO "hall: Executing WRITE.\n");

	switch (buf[0]) {
	case '0':
		hall_length = 0;
		break;
	case '1':
		hall_length = HALL_MAX;
		break;

	default:
		printk("hall: Wrong option.\n");
		break;
	}

	return count;
}

/*
 * / .open
 */
static int hall_open(struct inode *inode, struct file *file) {
	return 0;
}

/*
 * .close
 */
static int hall_close(struct inode *inode, struct file *file) {
	return 0;
}

static struct file_operations FileOps =
		{ .owner = THIS_MODULE, .open = hall_open, .read = hall_read, .write =
				hall_write, .release = hall_close, };

/*
 * Module init function.
 */
static int __init init_hall(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000);

	init_result = alloc_chrdev_region( &first, 0, 2, "hall_driver" );

	if( 0 > init_result )
	{
		printk( KERN_ALERT "hall: Device Registration failed\n" );
		return -1;
	}
	else
	{
		printk( KERN_ALERT "hall: Major number is: %d\n",init_result );
		//return 0;
	}

	if ( (cl = class_create( THIS_MODULE, "chardev" ) ) == NULL )
	{
		printk( KERN_ALERT "hall: Class creation failed\n" );
		unregister_chrdev_region( first, 1 );
		return -1;
	}

	if( device_create( cl, NULL, first, NULL, "hall" ) == NULL )
	{
		printk( KERN_ALERT "hall: Device creation failed\n" );
		class_destroy(cl);
		unregister_chrdev_region( first, 2 );
		return -1;
	}
	cdev_init( &c_dev, &FileOps1 );

	if( cdev_add( &c_dev, first, 1 ) == -1)
	{
		printk( KERN_ALERT "hall device addition failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 2 );
		return -1;
	}

	if(gpio_request(GPIO_HALLA, "halla"))
	{
		printk( KERN_ALERT "gpio request failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 2 );
		return -1;
	}
	if(gpio_request(GPIO_HALLB, "hallb"))
	{
		printk( KERN_ALERT "gpio request failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 2 );
		return -1;
	}

	if((hall_a_irq = gpio_to_irq(GPIO_HALLA)) < 0)
	{
		printk( KERN_ALERT "gpio to irq failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 2 );
		return -1;
	}
	if((hall_b_irq = gpio_to_irq(GPIO_HALLB)) < 0)
	{
		printk( KERN_ALERT "gpio to irq failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 2 );
		return -1;
	}

	if(request_irq(hall_a_irq, halla_rising_handler, IRQF_TRIGGER_RISING | IRQF_DISABLED, "gpiomod#hall", NULL ) == -1)
	{
		printk( KERN_ALERT "hall device interrupt handle failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 1 );

		return -1;
	}
	else
	{
		printk( KERN_ALERT "hall: Device irq number is %d\n", hall_a_irq );
	}
	if(request_irq(hall_a_irq, halla_falling_handler, IRQF_TRIGGER_FALLING | IRQF_DISABLED, "gpiomod#hall", NULL ) == -1)
	{
		printk( KERN_ALERT "hall device interrupt handle failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 1 );

		return -1;
	}
	else
	{
		printk( KERN_ALERT "hall: Device irq number is %d\n", hall_a_irq );
	}
	if(request_irq(hall_b_irq, hallb_rising_handler, IRQF_TRIGGER_RISING | IRQF_DISABLED, "gpiomod#hall", NULL ) == -1)
	{
		printk( KERN_ALERT "hall device interrupt handle failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 1 );

		return -1;
	}
	else
	{
		printk( KERN_ALERT "hall: Device irq number is %d\n", hall_b_irq );
	}
	if(request_irq(hall_b_irq, hallb_falling_handler, IRQF_TRIGGER_FALLING | IRQF_DISABLED, "gpiomod#hall", NULL ) == -1)
	{
		printk( KERN_ALERT "hall device interrupt handle failed\n" );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 1 );

		return -1;
	}
	else
	{
		printk( KERN_ALERT "hall: Device irq number is %d\n", hall_b_irq );
	}

	/*
	 * Module exit function
	 */
	static void __exit cleanup_hall(void)
	{
		cdev_del( &c_dev );
		device_destroy( cl, first );
		class_destroy( cl );
		unregister_chrdev_region( first, 1 );

		printk(KERN_ALERT "Hall: Device unregistered\n");

		free_irq(hall_a_irq, NULL);
		free_irq(hall_b_irq, NULL);
		gpio_free(GPIO_HALLA);
		gpio_free(GPIO_HALLB);
	}

	module_init( init_hall);
	module_exit( cleanup_hall);

	MODULE_AUTHOR("Bjorn Smith");
	MODULE_LICENSE("GPL");
	MODULE_DESCRIPTION("Actuator module monitoring hall signals from a Linak LA36");
