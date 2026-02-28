/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Max30102 SPO2 oximetry driver 
 * Copyright (C) 2026 Shi Hao <i.shihao.999@gmail.com>
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/buffer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define REG_INT_STATUS1				0x00
#define REG_INT_STATUS1_A_FULL			BIT(7)
#define REG_INT_STATUS1_PPG_RDY			BIT(6)
#define REG_INT_STATUS1_ALC_OVF			BIT(5)
#define REG_INT_STATUS1_PWR_RDY			BIT(0)
#define REG_INT_STATUS2				0x01
#define REG_INT_STATUS2_DIE_TEMP_RDY		BIT(1)

#define REG_INT_ENABLE1				0x02
#define REG_INT_ENABLE1_A_FULL			BIT(7)
#define REG_INT_EN1_A_FULL_SHIFT		6
#define REG_INT_ENABLE1_PPG_RDY_EN		BIT(6)
#define REG_INT_EN1_PPG_RDY_SHIFT		5
#define REG_INT_ENABLE1_ALC_OVF_EN		BIT(5)
#define REG_INT_EN1_ALC_OVF_SHIFT		4
#define REG_INT_ENABLE1_MASKS \
(REG_INT_ENABLE1_A_FULL \
| REG_INT_ENABLE1_PPG_RDY_EN  \
REG_INT_ENABLE1_ALC_OVF_EN)
#define REG_INT_ENABLE2				0x03
#define REG_INT_ENABLE2_DIE_TEMP_RDY_EN		BIT(1)
#define REG_INT_EN2_DIR_TEMP_RDY_SHIFT		1

#define REG_FIFO_SHIFT				0
#define REG_FIFO_WR_PTR				0x04
#define REG_FIFO_PTR_MASK			GENMASK(4,0)
#define REG_FIFO_CLEAR_PTR 			0x00
#define REG_FIFO_OVF_COUNTER			0x05
#define REG_FIFO_RD_PTR				0x06
#define REG_FIFO_DATA_REG			0x07

#define REG_FIFO_CONFIG				0x08
#define REG_FIFO_CONFIG_SMP_4AVE		0x02
#define REG_FIFO_CONFIG_SMP_AVE_SHIFT		5
#define REG_FIFO_CONFIG_SMP_AVE_MASK		GENMASK(7,5)
#define REG_FIFO_CONFIG_ROLLOVER_EN		BIT(4)
#define REG_FIFO_CONFIG_ROLLOVER_SHIFT		BIT(4)
#define REG_FIFO_CONFIG_ROLLOVER_MASK		BIT(4)
#define REG_FIFO_CONFIG_FIFO_A_FULL		0x06
#define REG_FIFO_CONFIG_FIFO_A_FULL_MASK	GENMASK(3,0)
#define REG_FIFO_CONFIG_FIFO_A_FULL_SHIFT	0
#define REG_FIFO_CONFIG_MASKS \
	(REG_FIFO_CONFIG_SMP_AVE_MASK | \
	 REG_FIFO_CONFIG_ROLLOVER_MASK| \
	 REG_FIFO_CONFIG_FIFO_A_FULL_MASK \
	 )

#define REG_MODE_CONFIG				0x09
#define REG_MODE_CONFIG_SHDN_SHIFT		7
#define REG_MODE_CONFIG_SHDN			BIT(7)
#define REG_MODE_CONFIG_RESET			BIT(6)
#define REG_MODE_CONFIG_RESET_SHIFT		6
#define REG_MODE_MASK				GENMASK(2,0)
#define REG_MODE_SHIFT				0

#define MODE_HR					0x02
#define MODE_SPO2				0x03
#define MODE_MULTI				0x07

#define REG_SPO2_CONFIG 			0x0A
#define REG_SPO2_ADC_SHIFT			5
#define REG_SPO2_ADC_MASK 			GENMASK(6,5)
#define REG_SPO2_SR_SHIFT 			2
#define REG_SPO2_SR_MASK  			GENMASK(4,2)
#define REG_SPO2_PW_SHIFT 			0
#define REG_SPO2_PW_MASK  			GENMASK(1,0)
#define REG_SPO2_CONFIG_MASK \
 (REG_SPO2_ADC_MASK | REG_SPO2_SR_MASK | REG_SPO2_PW_MASK)
#define SPO2_ADC_4096				1
#define SPO2_SR_100HZ				3
#define REG_MULTI_LED1_CONFIG			0x11
#define REG_MULTI_LED2_CONFIG			0x12
#define SPO2_PW_411US 				3

#define REG_LED1_PA				0x0C	/* RED */
#define REG_LED2_PA				0x0D	/* IR */
#define REG_LED1_DEF_PA				0x1f
#define REG_LED2_DEF_PA				0x1f
#define REG_TREG_INT				0x1F
#define REG_TFRAC				0x20
#define REG_TEMP_CONFIG				0x21
#define REG_TEMP_CONFIG_EN			BIT(0)
#define REG_REV_ID				0xFE
#define REG_PART_ID				0xFF
#define MAX30102_PART_NO			0x15
#define FIFO_MASK				0x1F
#define BYTES_PER_SAMPLE			6

#define MAX30102_INTENSITY_CHANNELS(_si,_mod) {\
	.type = IIO_INTENSITY,\
	.channel2 = _mod,\
	.modified = 1,\
	.scan_index = _si,\
	.scan_type = { \
		.sign = 'u',\
		.shift = 8, \
		.realbits = 18,\
		.storagebits = 32,\
		.endianness = IIO_BE,\
	},\
}\

struct max30102_data {
	struct regmap *regmap;
	struct iio_dev *indio_dev;
        struct work_struct work;
        struct gpio_desc *led_gpio;
        struct device *dev;
        bool device_state;
	u8 buffer[12];
	u8 bak_buf[192];
	__be32 pd[3];
	u8 intstatus;
	u8 intstatus1;
        int irq;

};

enum max30102_leds  {
	MAX_LED_RED,
	MAX_LED_IR,
};

static const unsigned long max30102_scan_masks[] = {
	BIT(MAX_LED_RED) | BIT(MAX_LED_IR),
	0
};

static const struct iio_chan_spec max30102_channels[] = {
	MAX30102_INTENSITY_CHANNELS(MAX_LED_RED,IIO_MOD_LIGHT_RED),
	MAX30102_INTENSITY_CHANNELS(MAX_LED_IR, IIO_MOD_LIGHT_IR),

};


static const struct regmap_config max30102_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int max30102_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val , int *val2, long mask)
{
        return 0;
}

static const struct iio_info max30102_iio_info = {
        .read_raw = max30102_read_raw,
};

int cal_unread_bytes(struct max30102_data *md)
{
	unsigned int r_value, w_value,
	int ret, samples;

	ret = regmap_read(md->regmap, REG_FIFO_WR_PTR,&w_value);

	if  (ret) {
		dev_err(md->dev, "remap_read() error");
		return ret;
	}

	ret = regmap_read(md->regmap, REG_FIFO_RD_PTR, &r_value);

	if (ret) {
		dev_err(md->dev, "regmap_read() error");
		return ret;
	}

	w_value &= FIFO_MASK;
        r_value &= FIFO_MASK;

	samples = (w_value - r_value) &  FIFO_MASK;;
	return  samples * BYTES_PER_SAMPLE;
}
static void max30102_workqueue(struct work_struct *work)
{
 	struct max30102_data *md = container_of(work, struct max30102_data , work);
        struct device *dev  = md->dev;

	/* calculate unread samples */
	int unread_bytes = cal_unread_bytes(md);

	if (unread_bytes < 0) {
		dev_err(md->dev, "cal_unread_bytes() error");
		return;
	}

	dev_info(md->dev, "unread_bytes :%d\n",unread_bytes);

	if (md->intstatus & REG_INT_STATUS1_A_FULL) {
	       dev_info(dev, "inturrpt A_FULL!");

	       /*TODO FIFO DATA register */
	ret = int regmap_bulk_read(md->regmap, REG_FIFO_DATA_REG, md->bak_buf,
				   unread_bytes);
	if (ret) {
		dev_err(dev, "regmap_bulk_read() error");
		return;
	}

	if (md->intstaus & REG_INT_STATUS1_PPG_RDY) {
		dev_info(dev, "inturrpt PPG_RDY!");

		ret = regmap_bulk_read(md->regmap, REG_FIFO_DATA_REG, md->buffer,
				unread_bytes);
		if (ret) {
			dev_err(dev, "regmap_bulk_read() error!");
			return;
		}
	}

	return;
}


static irqreturn_t max30102_irq_handler(int irq , void *dev_id)
{
	int ret;
	unsigned val;

	struct max30102_data *md = dev_id;

 	/* read both int status registers */

	dev_info(md->dev, "interrupt occured!\n");
	ret = regmap_read(md->regmap, REG_INT_STATUS1, &val);

	if (ret) {
		dev_err(md->dev, "regmap_read() error!");
	}

	md->intstatus = val;

	ret = regmap_read(md->regmap,REG_INT_STATUS2, &val);

	if (ret) {
		dev_err(md->dev, "regmap_read() error!");
	}

	md->intstatus1 = val;

	dev_info(md->dev, "scheduling workqueue");
	schedule_work(&md->work);

	return IRQ_HANDLED;
}

int max30102_chip_init(struct max30102_data *md)
{
	int ret ;
       	unsigned  int reg;
	
	/* Reset device */
	ret = regmap_update_bits(md->regmap, REG_MODE_CONFIG, REG_MODE_CONFIG_RESET,
			REG_MODE_CONFIG_RESET << REG_MODE_CONFIG_RESET_SHIFT);

	if (ret) {
		dev_err(md->dev, "reset failed!\n");
		return ret;
	}

	usleep_range(1000, 2000);

	/* read part id and revision */
	ret = regmap_read(md->regmap, REG_PART_ID, &reg);

	if (ret) {
		dev_err(md->dev, "part id read failed!\n");
		return ret;
	}


	if (reg != MAX30102_PART_NO) {
		dev_err(md->dev, "part id mismatch!\n");
		return ret;
	}

	dev_dbg(md->dev, "max30102 part id %02x\n", reg);

	ret = regmap_read(md->regmap, REG_REV_ID, &reg);

	if (ret) {
		dev_err(md->dev, "revision read failed\n");
		return ret;
	}

	dev_dbg(md->dev, "max30102 revision %02x\n", reg);

	/* put device in shutdown */
	ret = regmap_update_bits(md->regmap, REG_MODE_CONFIG, REG_MODE_CONFIG_SHDN , 
			REG_MODE_CONFIG_SHDN << REG_MODE_CONFIG_SHDN_SHIFT );

	if (ret) {
		dev_err(md->dev, "set mode shutdown failed!\n");
		return ret;
	}

	/* set power amplitude to both leds */
	ret = regmap_write(md->regmap, REG_LED1_PA, REG_LED1_DEF_PA);

	if (ret) {
		dev_err(md->dev, "set led1 PA failed!\n");
		return ret;
	}

	ret = regmap_write(md->regmap, REG_LED2_PA, REG_LED2_DEF_PA);

	if (ret) {
		dev_err(md->dev, "set led2 PA failed!\n");
		return ret;
	}

	/* FIFO configurations */
	ret = regmap_write(md->regmap,REG_FIFO_CONFIG,(REG_FIFO_CONFIG_SMP_4AVE
				<<REG_FIFO_CONFIG_SMP_AVE_SHIFT) |(
				REG_FIFO_CONFIG_ROLLOVER_EN 
				<< REG_FIFO_CONFIG_ROLLOVER_SHIFT) | (
				REG_FIFO_CONFIG_FIFO_A_FULL <<
				REG_FIFO_CONFIG_FIFO_A_FULL_SHIFT));
	if (ret) {
		dev_err(md->dev, "fifo configuration failed!\n");
		return ret;
	}

	/* spo2 configurations */
	ret = regmap_update_bits(md->regmap, REG_SPO2_CONFIG, REG_SPO2_CONFIG_MASK,
		       (SPO2_ADC_4096 << REG_SPO2_ADC_SHIFT) |
		       (SPO2_SR_100HZ << REG_SPO2_SR_SHIFT) |
		       (SPO2_PW_411US << REG_SPO2_PW_SHIFT));
	
	if (ret) {
		dev_err(md->dev, "spo2 configuration failed!\n");
       		return ret;
	}		

	/*  mode configuration */
	ret = regmap_update_bits(md->regmap, REG_MODE_CONFIG, REG_MODE_MASK,
			MODE_SPO2 << REG_MODE_SHIFT);
        if (ret) {
		dev_err(md->dev, "set spo2 mode failed!\n");
		return ret;
	}

	/* clear fifo data register */
	ret = regmap_write(md->regmap, REG_FIFO_WR_PTR,0);

        if (ret) {
		dev_err(md->dev, "fifo clear ptr failed!!\n");
		return ret;
	}

	ret = regmap_write(md->regmap, REG_FIFO_OVF_COUNTER,0);

        if (ret) {
		dev_err(md->dev, "fifo clear ptr failed!\n");
		return ret;
	}

	ret = regmap_write(md->regmap, REG_FIFO_RD_PTR, 0);

        if (ret) {
		dev_err(md->dev, "fifo clear ptr failed!\n");
		return ret;
	}

	/* interrupt configurations */
	ret = regmap_write(md->regmap, REG_INT_ENABLE1,(REG_INT_ENABLE1_A_FULL
			 << REG_INT_EN1_A_FULL_SHIFT) | (REG_INT_ENABLE1_PPG_RDY_EN
			 << REG_INT_EN1_PPG_RDY_SHIFT)| (REG_INT_ENABLE1_ALC_OVF_EN
			 << REG_INT_EN1_ALC_OVF_SHIFT));

	if (ret) {
		dev_err(md->dev, "interrupt1 enable failed!\n");
		return ret;
	}
	ret = regmap_write(md->regmap, REG_INT_ENABLE2 , REG_INT_ENABLE2_DIE_TEMP_RDY_EN
		       << REG_INT_EN2_DIR_TEMP_RDY_SHIFT);
	
	if (ret) { 
		dev_err(md->dev, "interrupt2 enable failed!\n");
		return ret;
	}
	
	/* clear SHDN bit to zero */ 
	ret = regmap_update_bits(md->regmap, REG_MODE_CONFIG, REG_MODE_CONFIG_SHDN,
			0 << REG_MODE_CONFIG_SHDN_SHIFT);

	if (ret) {
		dev_err(md->dev, "shdn clear failed!");
		return ret;
	}

	return 0;
}

static int max30102_probe(struct i2c_client *client)
{
	struct max30102_data *md;
	struct device *dev = &client->dev;
	int ret;

	md = devm_kzalloc(dev,sizeof(md), GFP_KERNEL);

	if (!md)
		return -ENOMEM;

	/* iio inteface configurations */
	md->indio_dev = devm_iio_device_alloc(dev, sizeof(struct max30102_data));

	if (!md->indio_dev) {
		dev_err(dev, "iio_device_alloc() error\n");
		return -ENOMEM;
	}

	md->indio_dev->info = &max30102_iio_info;
        md->indio_dev->channels = max30102_channels;
        md->indio_dev->num_channels = ARRAY_SIZE(max30102_channels);
        md->indio_dev->name  = "smd1306";
        md->indio_dev->available_scan_masks = max30102_scan_masks;
        md->indio_dev->modes  = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;

        ret = devm_iio_kfifo_buffer_setup(dev, md->indio_dev, NULL);

        if(ret < 0) {
                dev_err(dev,"iio_kfifo_buffer_setup() error\n");
                return -ret;
        }

        ret = devm_iio_device_register(dev, md->indio_dev);

        if (ret < 0 ) {
                dev_err(dev,"iio_device_register() error\n");
                return -ENODEV;
        }

        dev_info(dev, "iio registered\n");

	/* regmap configurations */
	
	md->regmap = devm_regmap_init_i2c(client, &max30102_regmap_config);
	if(IS_ERR(md->regmap))
		return  PTR_ERR(md->regmap);

	/* chip initialization */
	ret = max30102_chip_init(md);
	
	if (ret < 0) {
		dev_err(dev, "Initialization failed!\n");
		return ret;
	}

	/* interrupt configuration */
	md->irq = client->irq;

	ret = devm_request_irq(dev, md->irq, max30102_irq_handler, 0 , "max30102", md);

	if (ret) {
		dev_err(dev, "request_irq() error!");
		return ret;
	}

	INIT_WORK(&md->work, max30102_workqueue);

	pr_info("device ready\n");
	return 0;
}

static void max30102_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	
	dev_info(dev, "device removed\n");
	return;
}

static void max30102_shutdown(struct i2c_client *client)
{
	pr_info("Shutting down!\n");
	return;
}

static const struct of_device_id max30102_of_match[] = {
    { .compatible = "adi,max30102" },
    { }
};
MODULE_DEVICE_TABLE(of, max30102_of_match);

static const struct i2c_device_id max30102_idtable[] = {
	{ "adi,max30102"},
	{ }
};

MODULE_DEVICE_TABLE(i2c, max30102_idtable);

static struct i2c_driver max30102_driver = {
	.driver = {
		.name = "max30102",
		.pm = NULL,
		.of_match_table = max30102_of_match,
	},
	.id_table = max30102_idtable ,
	.probe = max30102_probe ,
	.remove = max30102_remove,
	.shutdown = max30102_shutdown,
};

static int __init max30102_init(void)
{
	int ret;

	ret = i2c_add_driver(&max30102_driver);

	return ret;
}

static void __exit max30102_exit(void)
{
	return i2c_del_driver(&max30102_driver);
}

module_init(max30102_init);
module_exit(max30102_exit);
MODULE_AUTHOR("Shi Hao <i.shihao.999@gmail.com>");
MODULE_DESCRIPTION("A simple max30102 oximeter sensor driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("max30102");
