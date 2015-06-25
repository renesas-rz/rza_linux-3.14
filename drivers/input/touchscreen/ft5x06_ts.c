/*
 * Driver for Focaltech FT5X06 Multiple Touch Controller
 *
 * Copyright (C) 2014 Renesas Electronics America.
 *
 * based on egalax_ts.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/io.h>  
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/input/mt.h>
#include <linux/timer.h>

//#define DB_PRINT

#ifdef DB_PRINT
#define DBG(...) printk("%s: ", __FUNCTION__);printk(__VA_ARGS__)
#else
#define DBG(...)
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5X06_SINGLE_TOUCH
#else
#define USE_ABS_MT
#endif

/* Focaltech FT5X06 register set */
#define WORK_MODE                       0x00
#define FACTORY_MODE                    0x04

#define ID_G_THGROUP		            0x80
#define ID_G_PERIODMONITOR	            0x89
#define FT5X0X_REG_HEIGHT_B	            0x8a
#define FT5X0X_REG_MAX_FRAME	        0x8b
#define FT5X0X_REG_FEG_FRAME	        0x8e
#define FT5X0X_REG_LEFT_RIGHT_OFFSET    0x92
#define FT5X0X_REG_UP_DOWN_OFFSET	    0x93
#define FT5X0X_REG_DISTANCE_LEFT_RIGHT	0x94
#define FT5X0X_REG_DISTANCE_UP_DOWN	    0x95
#define FT5X0X_REG_MAX_X_HIGH		    0x98
#define FT5X0X_REG_MAX_X_LOW		    0x99
#define FT5X0X_REG_MAX_Y_HIGH		    0x9a
#define FT5X0X_REG_MAX_Y_LOW		    0x9b
#define FT5X0X_REG_K_X_HIGH		        0x9c
#define FT5X0X_REG_K_X_LOW		        0x9d
#define FT5X0X_REG_K_Y_HIGH		        0x9e
#define FT5X0X_REG_K_Y_LOW		        0x9f

#define ID_G_AUTO_CLB	                0xa0
#define ID_G_B_AREA_TH	                0xae

//#define USE_ABS_MT 

struct point {
	int	x;
	int	y;
};

struct ft5x06_ts {
	struct i2c_client	*client;
	struct input_dev	*idev;
	struct delayed_work work;
	int irq;
	spinlock_t	        lock;
	struct point	last;
};
   
static struct workqueue_struct *ft5x06_ts_wq;

static int calibration[7] = {
	65536,0,0,
	0,65536,0,
	65536
};
module_param_array(calibration, int, NULL, S_IRUGO | S_IWUSR);

static int screenres[2] = {1280, 800};
module_param_array(screenres, int, NULL, S_IRUGO | S_IWUSR);

#ifndef CONFIG_MACH_RSKRZA1	/* not used for the RSK BSP */
static void translate(int *px, int *py)
{
	int x, y, x1, y1;
	if (calibration[6]) {
		x1 = *px;
		y1 = *py;

		x = calibration[0] * x1 +
			calibration[1] * y1 +
			calibration[2];
		x /= calibration[6];
		if (x < 0)
			x = 0;
		y = calibration[3] * x1 +
			calibration[4] * y1 +
			calibration[5];
		y /= calibration[6];
		if (y < 0)
			y = 0;
		*px = x ;
		*py = y ;
	}
}
#endif

static inline void ts_evt_add(struct ft5x06_ts *ts,
			                  unsigned buttons, struct point *p)
{
	struct input_dev *idev = ts->idev;
	int i;
	
	if (!buttons) { /* send release to user space. */
		/* Ignore duplicate reports */
		if( (ts->last.x == -1) && (ts->last.y == -1) )
			return;
		ts->last.x = -1;
		ts->last.y = -1;

#ifdef USE_ABS_MT
		input_event(idev, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
		input_event(idev, EV_KEY, BTN_TOUCH, 0);
		input_mt_sync(idev);
#else
		input_report_abs(idev, ABS_PRESSURE, 0);
		input_report_key(idev, BTN_TOUCH, 0);
		input_sync(idev);
#endif
	} else {
		for (i = 0; i < buttons; i++) {
#ifdef CONFIG_MACH_RSKRZA1
			/* The X and Y are inverted on this LCD panel */
			p[i].x = 800 - p[i].x;
			p[i].y = 480 - p[i].y;
#else
			translate(&p[i].x, &p[i].y);
#endif
			/* Ignore duplicates reports */
			if( (p[i].x == ts->last.x) && (p[i].y == ts->last.y))
				continue;
			ts->last.x = p[i].x;
			ts->last.y = p[i].y;
#ifdef USE_ABS_MT
			input_event(idev, EV_ABS, ABS_MT_POSITION_X, p[i].x);
			input_event(idev, EV_ABS, ABS_MT_POSITION_Y, p[i].y);
			input_event(idev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
			input_mt_sync(idev);
#else
			input_report_abs(idev, ABS_X, p[i].x);
			input_report_abs(idev, ABS_Y, p[i].y);
			input_report_abs(idev, ABS_PRESSURE, 1);
			input_report_key(idev, BTN_TOUCH, 1);
			input_sync(idev);
#endif
		}
		input_event(idev, EV_KEY, BTN_TOUCH, 1);
	}
#ifdef USE_ABS_MT
	input_sync(idev);
#endif
}

static void read_report(struct ft5x06_ts *ts)
{
	uint8_t  cmd = 0;
	uint8_t  buf[33];
	uint8_t  *p;
	int      ret, i, buttons = 0 ;
	struct point points[5];
	
	/* Clear buffer */
	memset(buf, 0, sizeof(buf));
	
	ret = i2c_master_send(ts->client, &cmd, 1);
	if (ret != 1) {
		dev_err(&ts->client->dev, "Unable to write to i2c touchscreen!\n");
		return;
	}		
	
	ret = i2c_master_recv(ts->client, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		dev_err(&ts->client->dev, "Unable to read i2c page!\n");		
		return;
	}	
		
	p = buf + 3; 
	buttons = buf[2] & 0x07;	/* Only look at the lower 3 bits because after power on,
					   sometimes the upper bits are not '0's until the first
					   physical touch */

	if (buttons > 5) {
		printk(KERN_ERR 
		       "%s: invalid button count %02x\n",
				__func__, buttons);
				buttons = 0 ;
	} else {
		for (i = 0; i < buttons; i++) {
			points[i].x = ((p[0] << 8) | p[1]) & 0x7ff;
			points[i].y = ((p[2] << 8) | p[3]) & 0x7ff;
			p += 6;
		}
	}	
	
	ts_evt_add(ts, buttons, points);	
}

static void ft5x06_ts_poscheck(struct work_struct *work)
{
	struct ft5x06_ts *ts = container_of(work, struct ft5x06_ts, work.work);

	read_report(ts);
	
	enable_irq(ts->client->irq);	
}

static irqreturn_t ft5x06_ts_isr(int irq, void *dev_id)
{     
	struct ft5x06_ts *ts = dev_id;

	disable_irq_nosync(irq);
		
	queue_work(ft5x06_ts_wq, &ts->work.work);
	
	return IRQ_HANDLED;	
}

#if 0
/* wake up controller by an falling edge of interrupt gpio.  */
static int ft5x06_wake_up_device(struct i2c_client *client)
{
	return 0;
}
#endif

static int ft5x06_write_reg(struct i2c_client *client, uint8_t reg, uint8_t val)
{
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = val;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static void ft5x06_set_mode(struct i2c_client *client, int mode)
{
	ft5x06_write_reg(client, 0, (mode & 7) << 4);
	printk(KERN_DEBUG "%s: changed mode to 0x%02x\n", __func__, mode);
}

static int ft5x06_detect(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	char buffer;
	struct i2c_msg pkt = {
		client->addr,
		I2C_M_RD,
		sizeof(buffer),
		&buffer
	};
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;
	if (i2c_transfer(adapter, &pkt, 1) != 1)
		return -ENODEV;

    DBG ("detect_ft5x06 sucessfully\n"); 	
	return 0;
}

static int ft5x06_init_panel(struct i2c_client *client)
{ 	
	int ret;
	
	ret = ft5x06_detect(client);
	
	if (ret != 0) {
		dev_err(&client->dev, "%s: could not detect ft5x06\n", __func__);
		return ret;
	}	
	
	ft5x06_set_mode(client, WORK_MODE);
	
	return 0;
}

static int ft5x06_ts_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
    struct ft5x06_ts *ts;
	struct input_dev *idev;
	int ret;
	int error;
		
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
					I2C_FUNC_SMBUS_READ_WORD_DATA  |
					I2C_FUNC_SMBUS_READ_I2C_BLOCK))
	{	
		dev_err(&client->dev, "Error bus incompatible\n");
		return -EIO;
	}

    /* allocate driver data struture */ 	
	ts = kzalloc(sizeof(struct ft5x06_ts), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	idev = input_allocate_device();
	if (!idev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_ts;
	}
	
	ts->client = client;
	ts->idev = idev;
	ts->irq = client->irq;
	ts->last.x = -1;
	ts->last.y = -1;

	idev->name = "FT5X06 Multiple Touch Controller";
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
     	
	__set_bit(EV_ABS, idev->evbit);
	__set_bit(EV_KEY, idev->evbit);
	__set_bit(BTN_TOUCH, idev->keybit);

#ifdef USE_ABS_MT
	input_set_abs_params(idev, ABS_MT_POSITION_X, 0, screenres[0]-1, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_Y, 0, screenres[1]-1, 0, 0);
	input_set_abs_params(idev, ABS_X, 0, screenres[0]-1, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, screenres[1]-1, 0, 0);
	input_set_abs_params(idev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
#else
	__set_bit(EV_SYN, idev->evbit);
	input_set_abs_params(idev, ABS_X, 0, screenres[0]-1, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, screenres[1]-1, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0);
#endif
		
	/* get idev */
    input_set_drvdata(idev, ts);
   
    /* Controller chip interrrupt active low enable */
	ret = ft5x06_init_panel(client);
	if (ret < 0) {
		dev_err(&client->dev, "could not init touch panel\n");
		error = -EIO;
		goto err_free_ts;
	}	
	
	/* Register input device */
	error = input_register_device(ts->idev);
	if (error)
		goto err_free_ts;

	i2c_set_clientdata(client, ts);
	
	spin_lock_init(&ts->lock);
	
	/* Create workqueue */
    ft5x06_ts_wq = create_singlethread_workqueue ("ft5x06_ts_wq");
	
    if(!ft5x06_ts_wq) {
	   error = -ENOMEM;
	   goto err_free_dev;
    }
	
	INIT_WORK(&ts->work.work, ft5x06_ts_poscheck);
   
    /* gpio irq handler register */
	error = request_irq(client->irq, ft5x06_ts_isr, 0, "ft5x06_ts", ts);
					
	if (error < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_wq;
	}
	
	device_init_wakeup(&client->dev, 1);
		
	return 0;

err_free_wq:
	destroy_workqueue(ft5x06_ts_wq);	
err_free_dev:
	input_free_device(idev);
err_free_ts:
	kfree(ts);
	return error;
}

static int ft5x06_ts_remove(struct i2c_client *client) 
{
	struct ft5x06_ts *ts = i2c_get_clientdata(client);

	if (ft5x06_ts_wq) {
	  flush_workqueue(ft5x06_ts_wq);
	  destroy_workqueue(ft5x06_ts_wq);
	}
	  
	free_irq(client->irq, ts);

	input_unregister_device(ts->idev);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{ "ft5x06-ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_PM_SLEEP
static int ft5x06_ts_suspend(struct device *dev) 
{
	struct i2c_client *client = to_i2c_client(dev);	
	struct ft5x06_ts *ts = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(ts->irq);
		
	return 0;
}

static int ft5x06_ts_resume(struct device *dev) 
{
	struct i2c_client *client = to_i2c_client(dev);	
	struct ft5x06_ts *ts = dev_get_drvdata(&client->dev);
	
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(ts->irq);	
	
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5x06_ts_pm_ops, ft5x06_ts_suspend, ft5x06_ts_resume);


static struct i2c_driver ft5x06_ts_driver = {
	.driver = {
		.name	= "ft5x06-ts",
		.owner	= THIS_MODULE,
		.pm	= &ft5x06_ts_pm_ops,
	},
	.id_table	= ft5x06_ts_id,
	.probe		= ft5x06_ts_probe,
	.remove		= ft5x06_ts_remove,
};

module_i2c_driver(ft5x06_ts_driver);

MODULE_AUTHOR("Renesas Electronics America");
MODULE_DESCRIPTION("Touchscreen driver for FT5X06 touch controller");
MODULE_LICENSE("GPL");
