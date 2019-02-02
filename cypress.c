/*
 *  Driver for Cypress Cy8c20466 Touchscreen Capsense Psoc
 *
 *  Originally from
 *  Copyright (c) 2014 Red Hat Inc.
 *  Copyright (c) 2015 K. Merker <merker@debian.org>
 *
 *  This code is based on gt9xx.c authored by andrew@cypress.com:
 *
 *  2019 ProCount
 */

#define DEBUG
#define TESTING 0

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <asm/unaligned.h>
#include <linux/jiffies.h>

/////////////////////////////// DEFINITIONS ////////////////////////////////

#define CYPRESS_GPIO_INT_NAME		"irq"

struct cypress_ts_data {
    struct i2c_client *client;
    struct input_dev *input_dev;
    int abs_x_max;
    int abs_y_max;
    bool swapped_x_y;
    bool inverted_x;
    bool inverted_y;
    unsigned int max_touch_num;
    unsigned int int_trigger_type;
    int cfg_len;
    struct gpio_desc *gpiod_int;
    struct gpio_desc *gpiod_rst;
    u16 id;
    u16 version;
    const char *cfg_name;
    struct completion firmware_loading_complete;
    unsigned long irq_flags;
    struct task_struct	*thread;
    
    int touch_one_start;
    int touch_two_start;
    int last_x1;
    int last_y1;
    int last_x2;
    int last_y2;

    bool X2Y;
    bool requestedX2Y;
    u16  requestedXSize;
    u16  requestedYSize;
    u8	 requestedRefreshRate;
};


/////////////////////////////// LOW LEVEL //////////////////////////////////

/**
 * cypress_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c Client info.
 * @reg: the register to read from.
 * @buf: raw read data buffer.
 * @len: length of the buffer to read
 * @returns 0, or -EIO if error
 */
static int cypress_i2c_read(struct i2c_client *client,
			   u16 reg, u8 *buf, int len)
{
    int bytes_read;

    bytes_read = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
    return (bytes_read != len ? -EIO : 0);
}

/**
 * cypress_i2c_write - write data to a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to write to.
 * @buf: raw data buffer to write.
 * @len: length of the buffer to write
 */
static int cypress_i2c_write(struct i2c_client *client, u16 reg, const u8 *buf,
			    unsigned len)
{
    int bytes_written;

    bytes_written = i2c_smbus_write_i2c_block_data(client, reg, len, buf);
    return (bytes_written != len ? -EIO : 0);
}

/////////////////////////////// PROCESSING /////////////////////////////////



/**
 * cypress_process_events - Process incoming events
 *
 * @ts: our cypress_ts_data pointer
 *
 * Called from poll or when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void cypress_process_events(struct cypress_ts_data *ts)
{
    int pin = desc_to_gpio(ts->gpiod_int);
    int value = gpiod_get_value(ts->gpiod_int);

    dev_dbg( &ts->client->dev, "process_events %d: %d\n",pin,value);

    if (gpiod_get_value(ts->gpiod_int))
    {   //Only process the TS if something has happened (or ...)

        //**BUT THIS SCOPE IS ALWAYS ENTERED!**
        u16 x1,y1,x2,y2;
        u8 rawdata[8];
        
        cypress_i2c_read(ts->client, 0x40, rawdata, 8);

        x1 = rawdata[0] | (rawdata[4] << 8);
        y1 = rawdata[1] | (rawdata[5] << 8);
        x2 = rawdata[2] | (rawdata[6] << 8);
        y2 = rawdata[3] | (rawdata[7] << 8);

        dev_dbg( &ts->client->dev, ".");
        if (x1 || y1)
            dev_dbg( &ts->client->dev, "x1,y1= %d, %d",x1,y1);
        if (x2 || y2)
            dev_dbg( &ts->client->dev, "x2,y2= %d, %d",x2,y2);

    }
}

/* Thread to poll for touchscreen events
 *
 */
static int cypress_thread(void *arg)
{

    while (!kthread_should_stop()) 
    {
        /* 60fps polling */
        struct cypress_ts_data *ts = (struct cypress_ts_data *) arg;

        /* Slowed down for debugging */
        msleep_interruptible(100); //17

       	cypress_process_events(ts);
	}

    return 0;
}

/////////////////////////////// DETECTION //////////////////////////////////


/**
 * cypress_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int cypress_i2c_test(struct i2c_client *client)
{
    /* Code to detect TS is fitted -ok */
    return 0;
}


/////////////////////////////// TERMINATION ////////////////////////////////

static int cypress_ts_remove(struct i2c_client *client)
{
    struct cypress_ts_data *ts = i2c_get_clientdata(client);

    if (ts->gpiod_int)
        devm_gpiod_put(&client->dev, ts->gpiod_int);

    return 0;
}

/////////////////////////////// INITIALISATION /////////////////////////////

/**
 * cypress_get_gpio_config - Get GPIO config from ACPI/DT
 *
 * @ts: cypress_ts_data pointer
 */
static int cypress_get_gpio_config(struct cypress_ts_data *ts)
{
    int error;
    struct device *dev;
    struct gpio_desc *gpiod;

    if (!ts->client)
        return -EINVAL;
    dev = &ts->client->dev;

    /* Get the interrupt GPIO pin description */
    gpiod = devm_gpiod_get_optional(dev, CYPRESS_GPIO_INT_NAME, GPIOD_IN);
    if (IS_ERR(gpiod)) {
        error = PTR_ERR(gpiod);
        if (error != -EPROBE_DEFER)
            dev_dbg(dev, "Failed to get %s GPIO: %d\n",
                CYPRESS_GPIO_INT_NAME, error);
        return error;
    }

    ts->gpiod_int = gpiod;
    return 0;
}

/**
 * cypress_request_input_dev - Allocate, populate and register the input device
 * @ts: our cypress_ts_data pointer
 * Must be called during probe
 */
static int cypress_request_input_dev(struct cypress_ts_data *ts)
{
int error;

ts->input_dev = devm_input_allocate_device(&ts->client->dev);
if (!ts->input_dev) {
	dev_err(&ts->client->dev, "Failed to allocate input device.");
	return -ENOMEM;
}

   /* Set up device parameters */
   /* ... */

    ts->input_dev->name = "Cypress Capacitive TouchScreen";
    ts->input_dev->phys = "input/ts";
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0x0416;
    ts->input_dev->id.product = ts->id;
    ts->input_dev->id.version = ts->version;

    error = input_register_device(ts->input_dev);
    if (error) {
        dev_err(&ts->client->dev,
        "Failed to register input device: %d", error);
    return error;
    }

    return 0;
}


static int cypress_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
    struct cypress_ts_data *ts;
    int error;
    u8 buf;

    dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C check functionality failed.\n");
        return -ENXIO;
    }

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts)
        return -ENOMEM;

    ts->client = client;
    i2c_set_clientdata(client, ts);

    buf = 0b00001110; //En_Int, Int_POLL, INT_MODE1
    cypress_i2c_write(client, 0x6e, &buf,1);
    error = cypress_i2c_test(client);
    if (error) {
        dev_err(&client->dev, "I2C communication failure: %d\n", error);
        return error;
    }

    cypress_request_input_dev(ts);

    cypress_get_gpio_config(ts);

    dev_dbg(&client->dev, "Cypress Cy8c20466 driver installed\n");

    /* create thread that polls the touch events */
    ts->thread = kthread_run(cypress_thread, ts, "cy82466?");
    if (ts->thread == NULL) {
        dev_err(&client->dev, "Failed to create kernel thread");
        //err = -ENOMEM;
        goto out;
    }
    return 0;
out:
    return -ENOMEM;
}

/////////////////////////////// ACPI ///////////////////////////////////////


/////////////////////////////// DEVICE DRIVER DATA /////////////////////////

static const struct i2c_device_id cypress_ts_id[] = {
    { "cy8c20466:00", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, cypress_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id cypress_of_match[] = {
    { .compatible = "cypress,cy8c20466" },
    { }
};
MODULE_DEVICE_TABLE(of, cypress_of_match);
#endif

static struct i2c_driver cypress_ts_driver = {
    .probe = cypress_ts_probe,
    .remove = cypress_ts_remove,
    .id_table = cypress_ts_id,
    .driver = {
        .name = "Cypress-TS",
        .of_match_table = of_match_ptr(cypress_of_match),
    },
};
module_i2c_driver(cypress_ts_driver);

MODULE_AUTHOR("procount <me@me.com>");
MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_DESCRIPTION("Cypress touchscreen driver");
MODULE_LICENSE("GPL v2");
