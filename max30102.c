#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>

static int __init max30102_init(void)
{
	return 0;
}

static void __exit max30102_exit(void)
{
	return;
}

module_init(max30102_init);
module_exit(max30102_exit);
MODULE_AUTHOR("Shi Hao <i.shihao.999@gmail.com>");
MODULE_DESCRIPTION("A simple max30102 oximeter sensor driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("max30102");

