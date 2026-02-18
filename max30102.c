#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

/* Register info */

#define INT_STATUS1				0x00
#define INT_STATUS1_A_FULL			BIT(7)
#define INT_STATUS1_PPG_RDY			BIT(6)
#define INT_STATUS1_ALC_OVF			BIT(5)
#define INT_STATUS1_PWR_RDY			BIT(0)
#define INT_STATUS2				0x01
#define INT_STATUS2_DIE_TEMP_RDY		BIT(1)

#define INT_ENABLE1				0x02
#define INT_ENABLE1_A_FULL			BIT(7)
#define INT_ENABLE1_PPG_RDY_EN			BIT(6)
#define INT_ENABLE1_ALC_OVF_EN			BIT(5)

#define INT_ENABLE2				0x03
#define INT_ENABLE2_DIE_TEMP_RDY_EN		BIT(1)

#define REG_FIFO_WR_PTR				0x04
#define REG_FIFO_OVF_COUNTER			0x05
#define REG_FIFO_RD_PTR				0x06
#define REG_FIFO_DATA_REG			0x07

#define REG_FIFO_CONFIG				0x08
#define REG_FIFO_CONFIG_SMP_4AVE		0x2
#define REG_FIFO_CONFIG_SMP_AVE_SHIFT		5
#define REG_FIFO_CONFIG_SMP_AVE_MASK		GENMASK(7,4)
#define REG_FIFO_CONFIG_ROLLOVER_EN		BIT(4)
#define REG_FIFO_CONFIG_FIFO_A_FULL		GENMASK(3,0)
#define REG_FIFO_CONFIG_FIFO_A_FULL_SHIFT	0


#define REG_MODE_CONFIG				0x09
#define REG_MODE_CONFIG_SHDN			BIT(7)
#define REG_MODE_CONFIG_RESET			BIT(6)

#define REG_MODE_MASK				GENMASK(2,0)
#define MODE_HR					0x02
#define MODE_SPO2				0x03
#define MODE_MULTI				0x07

#define REG_SPO2_CONFIG				0x0A
#define REG_SPO2_CONFIG_ADC_RGE_4096		(0x1 << 5)
#define REG_SPO2_CONFIG_SAMPLE_RATE		0x03
#define REG_SPO2_CONFIG_SR_MASK			GENMASK(4,2)
#define REG_SPO2_CONFIG_SR_SHIFT		2
#define REG_SPO2_LED_PW				0x03
#define REG_SPO2_LED_PW_SHIFT			0

#define REG_LED1_PA				0x0C	/* RED */
#define REG_LED2_PA				0x0D	/* IR */
#define REG_LED1_DEF_PA				0x1f
#define REG_LED2_DEF_PA				0x1f

#define REG_TINT				0x1F
#define REG_TFRAC				0x20
#define REG_TEMP_CONFIG				0x21
#define REG_TEMP_CONFIG_EN			BIT(0)

#define REG_REV_ID				0xFE
#define REG_PART_ID				0xFF

struct max30102_data {
	struct regmap *regmap;
}

static const struct regmap_config max30102_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
	cache_type = REGCACHE_RBTREE,
};


static int max30102_probe(struct i2c_client *client)
{
	struct max30102_data md;
	struct device *dev = &client->dev;
	unsigned int reg;
	md = devm_kzalloc(dev,sizeof(md), GFP_KERNEL);

	if (!md)
		return -ENOMEM;

	md->regmap = devm_regmap_init_i2c(client, &max30102_regmap_config);

	if(IS_ERR(md->regmap))
		return  PTR_ERR(map);

	/* check part ID */
	int val;
	ret = regmap_read(md->map,REG_PART_ID,&reg);

	if (ret < 0)
		return -ENODEV;
	if (reg != REG_PART_ID)
		return -ENODEV;

	pr_info("Driver initialized\n");
	return 0;
}

static void max30102_remove(struct i2c_client *client)
{
	pr_info("Driver removed\n");
	return;
}

static void max30102_shutdown(struct i2c_client *client)
{
	pr_info("Shutting down!\n");
	return;
}

static const struct i2c_device_id max30102_idtable[] = {
	{ "adi, max30102"},
	{ }
};

MODULE_DEVICE_TABLE(i2c, max30102_idtable);

static struct i2c_driver max30102_driver = {
	.driver = {
		.name = "max30102",
		.pm = NULL
	},
	.id_table = max30102_idtable ,
	.probe = max30102_probe ,
	.remove = max30102_remove,
	.shutdown = NULL
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
