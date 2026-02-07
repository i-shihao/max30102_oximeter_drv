#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>


static int max30102_probe(struct i2c_client *client)
{
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

