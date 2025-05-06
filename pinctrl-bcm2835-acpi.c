// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Broadcom BCM2835 GPIO unit (pinctrl + GPIO)
 *
 * Copyright (C) 2012 Chris Boot, Simon Arlott, Stephen Warren
 *
 * This driver is inspired by:
 * pinctrl-nomadik.c, please see original file for copyright information
 * pinctrl-tegra.c, please see original file for copyright information
 */

#include <linux/bitmap.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string_choices.h>
#include <linux/types.h>
#include <dt-bindings/pinctrl/bcm2835.h>
#include <linux/acpi.h>
#include <linux/pinctrl/machine.h>



#define MODULE_NAME "pinctrl-bcm2835-acpi"
#define BCM2835_NUM_GPIOS 54
#define BCM2711_NUM_GPIOS 58
#define BCM2835_NUM_BANKS 2
#define BCM2835_NUM_IRQS  3

/* GPIO register offsets */
#define GPFSEL0		0x0	/* Function Select */
#define GPSET0		0x1c	/* Pin Output Set */
#define GPCLR0		0x28	/* Pin Output Clear */
#define GPLEV0		0x34	/* Pin Level */
#define GPEDS0		0x40	/* Pin Event Detect Status */
#define GPREN0		0x4c	/* Pin Rising Edge Detect Enable */
#define GPFEN0		0x58	/* Pin Falling Edge Detect Enable */
#define GPHEN0		0x64	/* Pin High Detect Enable */
#define GPLEN0		0x70	/* Pin Low Detect Enable */
#define GPAREN0		0x7c	/* Pin Async Rising Edge Detect */
#define GPAFEN0		0x88	/* Pin Async Falling Edge Detect */
#define GPPUD		0x94	/* Pin Pull-up/down Enable */
#define GPPUDCLK0	0x98	/* Pin Pull-up/down Enable Clock */
#define GP_GPIO_PUP_PDN_CNTRL_REG0 0xe4 /* 2711 Pin Pull-up/down select */

#define FSEL_REG(p)		(GPFSEL0 + (((p) / 10) * 4))
#define FSEL_SHIFT(p)		(((p) % 10) * 3)
#define GPIO_REG_OFFSET(p)	((p) / 32)
#define GPIO_REG_SHIFT(p)	((p) % 32)

#define PUD_2711_MASK		0x3
#define PUD_2711_REG_OFFSET(p)	((p) / 16)
#define PUD_2711_REG_SHIFT(p)	(((p) % 16) * 2)

/* argument: bcm2835_pinconf_pull */
#define BCM2835_PINCONF_PARAM_PULL	(PIN_CONFIG_END + 1)

#define BCM2711_PULL_NONE	0x0
#define BCM2711_PULL_UP		0x1
#define BCM2711_PULL_DOWN	0x2


struct bcm2835_pinctrl {
	struct device *dev;
	void __iomem *base;
	int *wake_irq;

	/* note: locking assumes each bank will have its own unsigned long */
	unsigned long enabled_irq_map[BCM2835_NUM_BANKS];
	unsigned int irq_type[BCM2711_NUM_GPIOS];

	struct pinctrl_dev *pctl_dev;
	struct gpio_chip gpio_chip;
	struct pinctrl_desc pctl_desc;
	struct pinctrl_gpio_range gpio_range;

	raw_spinlock_t irq_lock[BCM2835_NUM_BANKS];
	/* Protect FSEL registers */
	spinlock_t fsel_lock;

	struct pinctrl_gpio_range i2c_range;
};



/* pins are just named GPIO0..GPIO53 */
#define BCM2835_GPIO_PIN(a) PINCTRL_PIN(a, "gpio" #a)
static struct pinctrl_pin_desc bcm2835_gpio_pins[] = {
	BCM2835_GPIO_PIN(0),
	BCM2835_GPIO_PIN(1),
	BCM2835_GPIO_PIN(2),
	BCM2835_GPIO_PIN(3),
	BCM2835_GPIO_PIN(4),
	BCM2835_GPIO_PIN(5),
	BCM2835_GPIO_PIN(6),
	BCM2835_GPIO_PIN(7),
	BCM2835_GPIO_PIN(8),
	BCM2835_GPIO_PIN(9),
	BCM2835_GPIO_PIN(10),
	BCM2835_GPIO_PIN(11),
	BCM2835_GPIO_PIN(12),
	BCM2835_GPIO_PIN(13),
	BCM2835_GPIO_PIN(14),
	BCM2835_GPIO_PIN(15),
	BCM2835_GPIO_PIN(16),
	BCM2835_GPIO_PIN(17),
	BCM2835_GPIO_PIN(18),
	BCM2835_GPIO_PIN(19),
	BCM2835_GPIO_PIN(20),
	BCM2835_GPIO_PIN(21),
	BCM2835_GPIO_PIN(22),
	BCM2835_GPIO_PIN(23),
	BCM2835_GPIO_PIN(24),
	BCM2835_GPIO_PIN(25),
	BCM2835_GPIO_PIN(26),
	BCM2835_GPIO_PIN(27),
	BCM2835_GPIO_PIN(28),
	BCM2835_GPIO_PIN(29),
	BCM2835_GPIO_PIN(30),
	BCM2835_GPIO_PIN(31),
	BCM2835_GPIO_PIN(32),
	BCM2835_GPIO_PIN(33),
	BCM2835_GPIO_PIN(34),
	BCM2835_GPIO_PIN(35),
	BCM2835_GPIO_PIN(36),
	BCM2835_GPIO_PIN(37),
	BCM2835_GPIO_PIN(38),
	BCM2835_GPIO_PIN(39),
	BCM2835_GPIO_PIN(40),
	BCM2835_GPIO_PIN(41),
	BCM2835_GPIO_PIN(42),
	BCM2835_GPIO_PIN(43),
	BCM2835_GPIO_PIN(44),
	BCM2835_GPIO_PIN(45),
	BCM2835_GPIO_PIN(46),
	BCM2835_GPIO_PIN(47),
	BCM2835_GPIO_PIN(48),
	BCM2835_GPIO_PIN(49),
	BCM2835_GPIO_PIN(50),
	BCM2835_GPIO_PIN(51),
	BCM2835_GPIO_PIN(52),
	BCM2835_GPIO_PIN(53),
	BCM2835_GPIO_PIN(54),
	BCM2835_GPIO_PIN(55),
	BCM2835_GPIO_PIN(56),
	BCM2835_GPIO_PIN(57),
};

/* one pin per group */
static const char * const bcm2835_gpio_groups[] = {
	"gpio0",
	"gpio1",
	"gpio2",
	"gpio3",
	"gpio4",
	"gpio5",
	"gpio6",
	"gpio7",
	"gpio8",
	"gpio9",
	"gpio10",
	"gpio11",
	"gpio12",
	"gpio13",
	"gpio14",
	"gpio15",
	"gpio16",
	"gpio17",
	"gpio18",
	"gpio19",
	"gpio20",
	"gpio21",
	"gpio22",
	"gpio23",
	"gpio24",
	"gpio25",
	"gpio26",
	"gpio27",
	"gpio28",
	"gpio29",
	"gpio30",
	"gpio31",
	"gpio32",
	"gpio33",
	"gpio34",
	"gpio35",
	"gpio36",
	"gpio37",
	"gpio38",
	"gpio39",
	"gpio40",
	"gpio41",
	"gpio42",
	"gpio43",
	"gpio44",
	"gpio45",
	"gpio46",
	"gpio47",
	"gpio48",
	"gpio49",
	"gpio50",
	"gpio51",
	"gpio52",
	"gpio53",
	"gpio54",
	"gpio55",
	"gpio56",
	"gpio57",
};

enum bcm2835_fsel {
	BCM2835_FSEL_COUNT = 8,
	BCM2835_FSEL_MASK = 0x7,
};

void bcm2835_bind_gpio_function(struct bcm2835_pinctrl *pc, unsigned int pin, enum bcm2835_fsel func);


static const char * const bcm2835_functions[BCM2835_FSEL_COUNT] = {
	[BCM2835_FSEL_GPIO_IN] = "gpio_in",
	[BCM2835_FSEL_GPIO_OUT] = "gpio_out",
	[BCM2835_FSEL_ALT0] = "alt0",
	[BCM2835_FSEL_ALT1] = "alt1",
	[BCM2835_FSEL_ALT2] = "alt2",
	[BCM2835_FSEL_ALT3] = "alt3",
	[BCM2835_FSEL_ALT4] = "alt4",
	[BCM2835_FSEL_ALT5] = "alt5",
};

static const char * const irq_type_names[] = {
	[IRQ_TYPE_NONE] = "none",
	[IRQ_TYPE_EDGE_RISING] = "edge-rising",
	[IRQ_TYPE_EDGE_FALLING] = "edge-falling",
	[IRQ_TYPE_EDGE_BOTH] = "edge-both",
	[IRQ_TYPE_LEVEL_HIGH] = "level-high",
	[IRQ_TYPE_LEVEL_LOW] = "level-low",
};





static inline int bcm2835_gpio_get_bit(struct bcm2835_pinctrl *pc, unsigned reg,
		unsigned bit)
{
	reg += GPIO_REG_OFFSET(bit) * 4;
	return (readl(pc->base + reg) >> GPIO_REG_SHIFT(bit)) & 1;
}

static inline void bcm2835_gpio_set_bit(struct bcm2835_pinctrl *pc,
	unsigned reg, unsigned bit)
{
	u32 mask = BIT(GPIO_REG_SHIFT(bit));
	u32 offset = reg + (GPIO_REG_OFFSET(bit) * 4);
	void __iomem *addr = pc->base + offset;

	writel(mask, addr);
}

static inline enum bcm2835_fsel bcm2835_pinctrl_fsel_get(
		struct bcm2835_pinctrl *pc, unsigned pin)
{
	u32 val = readl(pc->base + FSEL_REG(pin));
	enum bcm2835_fsel status = (val >> FSEL_SHIFT(pin)) & BCM2835_FSEL_MASK;

	return status;
}

static inline void bcm2835_pinctrl_fsel_set(
		struct bcm2835_pinctrl *pc, unsigned pin,
		enum bcm2835_fsel fsel)
{
	u32 val;
	enum bcm2835_fsel cur;
	unsigned long flags;

	spin_lock_irqsave(&pc->fsel_lock, flags);
	val = readl(pc->base + FSEL_REG(pin));
	cur = (val >> FSEL_SHIFT(pin)) & BCM2835_FSEL_MASK;

	if (cur == fsel)
		goto unlock;

	if (cur != BCM2835_FSEL_GPIO_IN && fsel != BCM2835_FSEL_GPIO_IN) {
		val &= ~(BCM2835_FSEL_MASK << FSEL_SHIFT(pin));
		val |= BCM2835_FSEL_GPIO_IN << FSEL_SHIFT(pin);
		writel(val, pc->base + FSEL_REG(pin));
	}

	val &= ~(BCM2835_FSEL_MASK << FSEL_SHIFT(pin));
	val |= fsel << FSEL_SHIFT(pin);
	writel(val, pc->base + FSEL_REG(pin));

unlock:
	spin_unlock_irqrestore(&pc->fsel_lock, flags);
}

static int bcm2835_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);

	bcm2835_pinctrl_fsel_set(pc, offset, BCM2835_FSEL_GPIO_IN);
	return 0;
}

static int bcm2835_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);

	return bcm2835_gpio_get_bit(pc, GPLEV0, offset);
}

static int bcm2835_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);
	enum bcm2835_fsel fsel = bcm2835_pinctrl_fsel_get(pc, offset);

	if (fsel == BCM2835_FSEL_GPIO_OUT)
		return GPIO_LINE_DIRECTION_OUT;

	/*
	 * Alternative function doesn't clearly provide a direction. Default
	 * to INPUT.
	 */
	return GPIO_LINE_DIRECTION_IN;
}

static void bcm2835_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);

	bcm2835_gpio_set_bit(pc, value ? GPSET0 : GPCLR0, offset);
}

static int bcm2835_gpio_direction_output(struct gpio_chip *chip,
		unsigned offset, int value)
{
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);

	bcm2835_gpio_set_bit(pc, value ? GPSET0 : GPCLR0, offset);
	bcm2835_pinctrl_fsel_set(pc, offset, BCM2835_FSEL_GPIO_OUT);
	return 0;
}



static const struct gpio_chip bcm2835_gpio_chip = {
	.label = MODULE_NAME,
	.owner = THIS_MODULE,
	.request = gpiochip_generic_request,
	.free = gpiochip_generic_free,
	.direction_input = bcm2835_gpio_direction_input,
	.direction_output = bcm2835_gpio_direction_output,
	.get_direction = bcm2835_gpio_get_direction,
	.get = bcm2835_gpio_get,
	.set = bcm2835_gpio_set,
	.set_config = gpiochip_generic_config,
	.base = -1,
	.ngpio = BCM2835_NUM_GPIOS,
	.can_sleep = false,
};

static const struct gpio_chip bcm2711_gpio_chip = {
	.label = "pinctrl-bcm2711",
	.owner = THIS_MODULE,
	.request = gpiochip_generic_request,
	.free = NULL,
	.direction_input = bcm2835_gpio_direction_input,
	.direction_output = bcm2835_gpio_direction_output,
	.get_direction = bcm2835_gpio_get_direction,
	.get = bcm2835_gpio_get,
	.set = bcm2835_gpio_set,
	.set_config = gpiochip_generic_config,
	.base = -1,
	.ngpio = BCM2711_NUM_GPIOS,
	.can_sleep = false,
};



static irqreturn_t bcm2835_gpio_wake_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static inline void __bcm2835_gpio_irq_config(struct bcm2835_pinctrl *pc,
	unsigned reg, unsigned offset, bool enable)
{
	u32 value;
	reg += GPIO_REG_OFFSET(offset) * 4;
	value = readl(pc->base + reg);
	if (enable)
		value |= BIT(GPIO_REG_SHIFT(offset));
	else
		value &= ~(BIT(GPIO_REG_SHIFT(offset)));
	writel(value, pc->base + reg);
}

/* fast path for IRQ handler */
static void bcm2835_gpio_irq_config(struct bcm2835_pinctrl *pc,
	unsigned offset, bool enable)
{
	switch (pc->irq_type[offset]) {
	case IRQ_TYPE_EDGE_RISING:
		__bcm2835_gpio_irq_config(pc, GPREN0, offset, enable);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		__bcm2835_gpio_irq_config(pc, GPFEN0, offset, enable);
		break;

	case IRQ_TYPE_EDGE_BOTH:
		__bcm2835_gpio_irq_config(pc, GPREN0, offset, enable);
		__bcm2835_gpio_irq_config(pc, GPFEN0, offset, enable);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		__bcm2835_gpio_irq_config(pc, GPHEN0, offset, enable);
		break;

	case IRQ_TYPE_LEVEL_LOW:
		__bcm2835_gpio_irq_config(pc, GPLEN0, offset, enable);
		break;
	}
}

static void bcm2835_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);
	unsigned gpio = irqd_to_hwirq(data);
	unsigned offset = GPIO_REG_SHIFT(gpio);
	unsigned bank = GPIO_REG_OFFSET(gpio);
	unsigned long flags;

	gpiochip_enable_irq(chip, gpio);

	raw_spin_lock_irqsave(&pc->irq_lock[bank], flags);
	set_bit(offset, &pc->enabled_irq_map[bank]);
	bcm2835_gpio_irq_config(pc, gpio, true);
	raw_spin_unlock_irqrestore(&pc->irq_lock[bank], flags);
}

static void bcm2835_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);
	unsigned gpio = irqd_to_hwirq(data);
	unsigned offset = GPIO_REG_SHIFT(gpio);
	unsigned bank = GPIO_REG_OFFSET(gpio);
	unsigned long flags;

	raw_spin_lock_irqsave(&pc->irq_lock[bank], flags);
	bcm2835_gpio_irq_config(pc, gpio, false);
	/* Clear events that were latched prior to clearing event sources */
	bcm2835_gpio_set_bit(pc, GPEDS0, gpio);
	clear_bit(offset, &pc->enabled_irq_map[bank]);
	raw_spin_unlock_irqrestore(&pc->irq_lock[bank], flags);

	gpiochip_disable_irq(chip, gpio);
}

static int __bcm2835_gpio_irq_set_type_disabled(struct bcm2835_pinctrl *pc,
	unsigned offset, unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_NONE:
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_BOTH:
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		pc->irq_type[offset] = type;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/* slower path for reconfiguring IRQ type */
static int __bcm2835_gpio_irq_set_type_enabled(struct bcm2835_pinctrl *pc,
	unsigned offset, unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_NONE:
		if (pc->irq_type[offset] != type) {
			bcm2835_gpio_irq_config(pc, offset, false);
			pc->irq_type[offset] = type;
		}
		break;

	case IRQ_TYPE_EDGE_RISING:
		if (pc->irq_type[offset] == IRQ_TYPE_EDGE_BOTH) {
			/* RISING already enabled, disable FALLING */
			pc->irq_type[offset] = IRQ_TYPE_EDGE_FALLING;
			bcm2835_gpio_irq_config(pc, offset, false);
			pc->irq_type[offset] = type;
		} else if (pc->irq_type[offset] != type) {
			bcm2835_gpio_irq_config(pc, offset, false);
			pc->irq_type[offset] = type;
			bcm2835_gpio_irq_config(pc, offset, true);
		}
		break;

	case IRQ_TYPE_EDGE_FALLING:
		if (pc->irq_type[offset] == IRQ_TYPE_EDGE_BOTH) {
			/* FALLING already enabled, disable RISING */
			pc->irq_type[offset] = IRQ_TYPE_EDGE_RISING;
			bcm2835_gpio_irq_config(pc, offset, false);
			pc->irq_type[offset] = type;
		} else if (pc->irq_type[offset] != type) {
			bcm2835_gpio_irq_config(pc, offset, false);
			pc->irq_type[offset] = type;
			bcm2835_gpio_irq_config(pc, offset, true);
		}
		break;

	case IRQ_TYPE_EDGE_BOTH:
		if (pc->irq_type[offset] == IRQ_TYPE_EDGE_RISING) {
			/* RISING already enabled, enable FALLING too */
			pc->irq_type[offset] = IRQ_TYPE_EDGE_FALLING;
			bcm2835_gpio_irq_config(pc, offset, true);
			pc->irq_type[offset] = type;
		} else if (pc->irq_type[offset] == IRQ_TYPE_EDGE_FALLING) {
			/* FALLING already enabled, enable RISING too */
			pc->irq_type[offset] = IRQ_TYPE_EDGE_RISING;
			bcm2835_gpio_irq_config(pc, offset, true);
			pc->irq_type[offset] = type;
		} else if (pc->irq_type[offset] != type) {
			bcm2835_gpio_irq_config(pc, offset, false);
			pc->irq_type[offset] = type;
			bcm2835_gpio_irq_config(pc, offset, true);
		}
		break;

	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		if (pc->irq_type[offset] != type) {
			bcm2835_gpio_irq_config(pc, offset, false);
			pc->irq_type[offset] = type;
			bcm2835_gpio_irq_config(pc, offset, true);
		}
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int bcm2835_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);
	unsigned gpio = irqd_to_hwirq(data);
	unsigned offset = GPIO_REG_SHIFT(gpio);
	unsigned bank = GPIO_REG_OFFSET(gpio);
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&pc->irq_lock[bank], flags);

	if (test_bit(offset, &pc->enabled_irq_map[bank]))
		ret = __bcm2835_gpio_irq_set_type_enabled(pc, gpio, type);
	else
		ret = __bcm2835_gpio_irq_set_type_disabled(pc, gpio, type);

	if (type & IRQ_TYPE_EDGE_BOTH)
		irq_set_handler_locked(data, handle_edge_irq);
	else
		irq_set_handler_locked(data, handle_level_irq);

	raw_spin_unlock_irqrestore(&pc->irq_lock[bank], flags);

	return ret;
}

static void bcm2835_gpio_irq_ack(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);
	unsigned gpio = irqd_to_hwirq(data);

	bcm2835_gpio_set_bit(pc, GPEDS0, gpio);
}

static int bcm2835_gpio_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);
	unsigned gpio = irqd_to_hwirq(data);
	unsigned int irqgroup;
	int ret = -EINVAL;

	if (!pc->wake_irq)
		return ret;

	if (gpio <= 27)
		irqgroup = 0;
	else if (gpio >= 28 && gpio <= 45)
		irqgroup = 1;
	else if (gpio >= 46 && gpio <= 57)
		irqgroup = 2;
	else
		return ret;

	if (on)
		ret = enable_irq_wake(pc->wake_irq[irqgroup]);
	else
		ret = disable_irq_wake(pc->wake_irq[irqgroup]);

	return ret;
}

static const struct irq_chip bcm2835_gpio_irq_chip = {
	.name = MODULE_NAME,
	.irq_set_type = bcm2835_gpio_irq_set_type,
	.irq_ack = bcm2835_gpio_irq_ack,
	.irq_mask = bcm2835_gpio_irq_mask,
	.irq_unmask = bcm2835_gpio_irq_unmask,
	.irq_set_wake = bcm2835_gpio_irq_set_wake,
	.flags = (IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_IMMUTABLE),
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int bcm2835_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return BCM2835_NUM_GPIOS;
}

static const char *bcm2835_pctl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned selector)
{
	return bcm2835_gpio_groups[selector];
}

static int bcm2835_pctl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned selector,
		const unsigned **pins,
		unsigned *num_pins)
{
	*pins = &bcm2835_gpio_pins[selector].number;
	*num_pins = 1;

	return 0;
}

static void bcm2835_pctl_pin_dbg_show(struct pinctrl_dev *pctldev,
		struct seq_file *s,
		unsigned offset)
{
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct gpio_chip *chip = &pc->gpio_chip;
	enum bcm2835_fsel fsel = bcm2835_pinctrl_fsel_get(pc, offset);
	const char *fname = bcm2835_functions[fsel];
	int value = bcm2835_gpio_get_bit(pc, GPLEV0, offset);
	int irq = irq_find_mapping(chip->irq.domain, offset);

	seq_printf(s, "function %s in %s; irq %d (%s)",
		fname, str_hi_lo(value),
		irq, irq_type_names[pc->irq_type[offset]]);
}

static void bcm2835_pctl_dt_free_map(struct pinctrl_dev *pctldev,
		struct pinctrl_map *maps, unsigned num_maps)
{
	int i;

	for (i = 0; i < num_maps; i++)
		if (maps[i].type == PIN_MAP_TYPE_CONFIGS_PIN)
			kfree(maps[i].data.configs.configs);

	kfree(maps);
}

static int bcm2835_pctl_dt_node_to_map_func(struct bcm2835_pinctrl *pc,
		struct device_node *np, u32 pin, u32 fnum,
		struct pinctrl_map **maps)
{
	struct pinctrl_map *map = *maps;

	if (fnum >= ARRAY_SIZE(bcm2835_functions)) {
		dev_err(pc->dev, "%pOF: invalid brcm,function %d\n", np, fnum);
		return -EINVAL;
	}

	map->type = PIN_MAP_TYPE_MUX_GROUP;
	map->data.mux.group = bcm2835_gpio_groups[pin];
	map->data.mux.function = bcm2835_functions[fnum];
	(*maps)++;

	return 0;
}

static int bcm2835_pctl_dt_node_to_map_pull(struct bcm2835_pinctrl *pc,
		struct device_node *np, u32 pin, u32 pull,
		struct pinctrl_map **maps)
{
	struct pinctrl_map *map = *maps;
	unsigned long *configs;

	if (pull > 2) {
		dev_err(pc->dev, "%pOF: invalid brcm,pull %d\n", np, pull);
		return -EINVAL;
	}

	configs = kzalloc(sizeof(*configs), GFP_KERNEL);
	if (!configs)
		return -ENOMEM;
	configs[0] = pinconf_to_config_packed(BCM2835_PINCONF_PARAM_PULL, pull);

	map->type = PIN_MAP_TYPE_CONFIGS_PIN;
	map->data.configs.group_or_pin = bcm2835_gpio_pins[pin].name;
	map->data.configs.configs = configs;
	map->data.configs.num_configs = 1;
	(*maps)++;

	return 0;
}

static int bcm2835_pctl_dt_node_to_map(struct pinctrl_dev *pctldev,
		struct device_node *np,
		struct pinctrl_map **map, unsigned int *num_maps)
{
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct property *pins, *funcs, *pulls;
	int num_pins, num_funcs, num_pulls, maps_per_pin;
	struct pinctrl_map *maps, *cur_map;
	int i, err;
	u32 pin, func, pull;

	dev_info(pc->dev, "Processing pinctrl bindings for node: %pOF\n", np);

	/* Check for generic binding in this node */
	err = pinconf_generic_dt_node_to_map_all(pctldev, np, map, num_maps);
	if (err || *num_maps)
		return err;

	/* Generic binding did not find anything continue with legacy parse */
	pins = of_find_property(np, "brcm,pins", NULL);
	if (!pins) {
		dev_err(pc->dev, "%pOF: missing brcm,pins property\n", np);
		return -EINVAL;
	}

	funcs = of_find_property(np, "brcm,function", NULL);
	pulls = of_find_property(np, "brcm,pull", NULL);

	if (!funcs && !pulls) {
		dev_err(pc->dev,
			"%pOF: neither brcm,function nor brcm,pull specified\n",
			np);
		return -EINVAL;
	}

	num_pins = pins->length / 4;
	num_funcs = funcs ? (funcs->length / 4) : 0;
	num_pulls = pulls ? (pulls->length / 4) : 0;

	if (num_funcs > 1 && num_funcs != num_pins) {
		dev_err(pc->dev,
			"%pOF: brcm,function must have 1 or %d entries\n",
			np, num_pins);
		return -EINVAL;
	}

	if (num_pulls > 1 && num_pulls != num_pins) {
		dev_err(pc->dev,
			"%pOF: brcm,pull must have 1 or %d entries\n",
			np, num_pins);
		return -EINVAL;
	}

	maps_per_pin = 0;
	if (num_funcs)
		maps_per_pin++;
	if (num_pulls)
		maps_per_pin++;
	cur_map = maps = kcalloc(num_pins * maps_per_pin, sizeof(*maps),
				 GFP_KERNEL);
	if (!maps)
		return -ENOMEM;

	for (i = 0; i < num_pins; i++) {
		err = of_property_read_u32_index(np, "brcm,pins", i, &pin);
		if (err)
			goto out;
		if (pin >= pc->pctl_desc.npins) {
			dev_err(pc->dev, "%pOF: invalid brcm,pins value %d\n",
				np, pin);
			err = -EINVAL;
			goto out;
		}

		dev_info(pc->dev, "Configuring pin %d\n", pin);

		if (num_funcs) {
			err = of_property_read_u32_index(np, "brcm,function",
					(num_funcs > 1) ? i : 0, &func);
			if (err)
				goto out;
			dev_info(pc->dev, "Setting function %d for pin %d\n", func, pin);
			err = bcm2835_pctl_dt_node_to_map_func(pc, np, pin,
							func, &cur_map);
			if (err)
				goto out;
		}
		if (num_pulls) {
			err = of_property_read_u32_index(np, "brcm,pull",
					(num_pulls > 1) ? i : 0, &pull);
			if (err)
				goto out;
			dev_info(pc->dev, "Setting pull %d for pin %d\n", pull, pin);
			err = bcm2835_pctl_dt_node_to_map_pull(pc, np, pin,
							pull, &cur_map);
			if (err)
				goto out;
		}
	}

	*map = maps;
	*num_maps = num_pins * maps_per_pin;

	return 0;

out:
	bcm2835_pctl_dt_free_map(pctldev, maps, num_pins * maps_per_pin);
	return err;
}

static const struct pinctrl_ops bcm2835_pctl_ops = {
	.get_groups_count = bcm2835_pctl_get_groups_count,
	.get_group_name = bcm2835_pctl_get_group_name,
	.get_group_pins = bcm2835_pctl_get_group_pins,
	.pin_dbg_show = bcm2835_pctl_pin_dbg_show,
	.dt_node_to_map = bcm2835_pctl_dt_node_to_map,
	.dt_free_map = bcm2835_pctl_dt_free_map,
};


static int bcm2835_pmx_get_functions_count(struct pinctrl_dev *pctldev)
{
	return BCM2835_FSEL_COUNT;
}

static const char *bcm2835_pmx_get_function_name(struct pinctrl_dev *pctldev,
		unsigned selector)
{
	return bcm2835_functions[selector];
}

static int bcm2835_pmx_get_function_groups(struct pinctrl_dev *pctldev,
		unsigned selector,
		const char * const **groups,
		unsigned * const num_groups)
{
	/* every pin can do every function */
	*groups = bcm2835_gpio_groups;
	*num_groups = BCM2835_NUM_GPIOS;

	return 0;
}

void bcm2835_bind_gpio_function(struct bcm2835_pinctrl *pc, unsigned pin, enum bcm2835_fsel func)
{
	bcm2835_pinctrl_fsel_set(pc, pin, func);
}

static int bcm2835_pmx_set(struct pinctrl_dev *pctldev,
		unsigned func_selector,
		unsigned group_selector)
{
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	dev_info(pc->dev, "Set mux: group=%s function=%s\n",
		bcm2835_gpio_groups[group_selector],
		bcm2835_functions[func_selector]);

   bcm2835_bind_gpio_function(pc, group_selector, func_selector);
   return 0;
}

// Add the persist_gpio_outputs parameter
static bool persist_gpio_outputs;
module_param(persist_gpio_outputs, bool, 0444);
MODULE_PARM_DESC(persist_gpio_outputs, "Enable GPIO_OUT persistence when pin is freed");

// Modify bcm2835_pmx_free to handle persist_gpio_outputs
static int bcm2835_pmx_free(struct pinctrl_dev *pctldev, unsigned offset)
{
    struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
    enum bcm2835_fsel fsel = bcm2835_pinctrl_fsel_get(pc, offset);

    if (fsel == BCM2835_FSEL_GPIO_IN)
        return 0;

    if (persist_gpio_outputs && fsel == BCM2835_FSEL_GPIO_OUT)
        return 0;

    /* Disable by setting to GPIO_IN */
    bcm2835_pinctrl_fsel_set(pc, offset, BCM2835_FSEL_GPIO_IN);
    return 0;
}

static int bcm2835_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned offset,
		bool input)
{
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum bcm2835_fsel fsel = input ?
		BCM2835_FSEL_GPIO_IN : BCM2835_FSEL_GPIO_OUT;

	bcm2835_pinctrl_fsel_set(pc, offset, fsel);

	return 0;
}

static int bcm2835_pmx_gpio_request_enable(struct pinctrl_dev *pctldev,
	struct pinctrl_gpio_range *range,
	unsigned offset)
{
struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

/* Set the pin to GPIO mode */
bcm2835_pinctrl_fsel_set(pc, offset, BCM2835_FSEL_GPIO_IN);
return 0;
}

static void bcm2835_pmx_gpio_disable_free(struct pinctrl_dev *pctldev,
   struct pinctrl_gpio_range *range,
   unsigned offset)
{
	bcm2835_pmx_free(pctldev, offset);
}

static const struct pinmux_ops bcm2835_pmx_ops = {
	.free = bcm2835_pmx_free,

	.get_functions_count = bcm2835_pmx_get_functions_count,
	.get_function_name = bcm2835_pmx_get_function_name,
	.get_function_groups = bcm2835_pmx_get_function_groups,
	.set_mux = bcm2835_pmx_set,
	.gpio_set_direction = bcm2835_pmx_gpio_set_direction,
	.gpio_request_enable = bcm2835_pmx_gpio_request_enable, 
    .gpio_disable_free = bcm2835_pmx_gpio_disable_free,     
};

static int bcm2835_pinconf_get(struct pinctrl_dev *pctldev,
			unsigned pin, unsigned long *config)
{
	enum pin_config_param param = pinconf_to_config_param(*config);
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum bcm2835_fsel fsel = bcm2835_pinctrl_fsel_get(pc, pin);
	u32 val;

	/* No way to read back bias config in HW */

	switch (param) {
	case PIN_CONFIG_OUTPUT:
		if (fsel != BCM2835_FSEL_GPIO_OUT)
			return -EINVAL;

		val = bcm2835_gpio_get_bit(pc, GPLEV0, pin);
		*config = pinconf_to_config_packed(param, val);
		break;

	default:
		return -ENOTSUPP;
	}

	return 0;
}

static void bcm2835_pull_config_set(struct bcm2835_pinctrl *pc,
		unsigned int pin, unsigned int arg)
{
	u32 off, bit;

	off = GPIO_REG_OFFSET(pin);
	bit = GPIO_REG_SHIFT(pin);

	writel(arg & 3, pc->base + GPPUD);
	/*
	 * BCM2835 datasheet say to wait 150 cycles, but not of what.
	 * But the VideoCore firmware delay for this operation
	 * based nearly on the same amount of VPU cycles and this clock
	 * runs at 250 MHz.
	 */
	udelay(1);
	writel(BIT(bit), pc->base + GPPUDCLK0 + (off * 4));
	udelay(1);
	writel(0, pc->base + GPPUDCLK0 + (off * 4));
}

static int bcm2835_pinconf_set(struct pinctrl_dev *pctldev,
			unsigned int pin, unsigned long *configs,
			unsigned int num_configs)
{
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	u32 param, arg;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		/* Set legacy brcm,pull */
		case BCM2835_PINCONF_PARAM_PULL:
			bcm2835_pull_config_set(pc, pin, arg);
			break;

		/* Set pull generic bindings */
		case PIN_CONFIG_BIAS_DISABLE:
			bcm2835_pull_config_set(pc, pin, BCM2835_PUD_OFF);
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			bcm2835_pull_config_set(pc, pin, BCM2835_PUD_DOWN);
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			bcm2835_pull_config_set(pc, pin, BCM2835_PUD_UP);
			break;

		/* Set output-high or output-low */
		case PIN_CONFIG_OUTPUT:
			bcm2835_gpio_set_bit(pc, arg ? GPSET0 : GPCLR0, pin);
			break;

		default:
			return -ENOTSUPP;

		} /* switch param type */
	} /* for each config */

	return 0;
}

static const struct pinconf_ops bcm2835_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = bcm2835_pinconf_get,
	.pin_config_set = bcm2835_pinconf_set,
};

static int bcm2711_pinconf_get(struct pinctrl_dev *pctldev, unsigned pin,
			       unsigned long *config)
{
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 offset, shift, val;

	offset = PUD_2711_REG_OFFSET(pin);
	shift = PUD_2711_REG_SHIFT(pin);
	val = readl(pc->base + GP_GPIO_PUP_PDN_CNTRL_REG0 + (offset * 4));

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (((val >> shift) & PUD_2711_MASK) != BCM2711_PULL_NONE)
			return -EINVAL;

		break;

	case PIN_CONFIG_BIAS_PULL_UP:
		if (((val >> shift) & PUD_2711_MASK) != BCM2711_PULL_UP)
			return -EINVAL;

		*config = pinconf_to_config_packed(param, 50000);
		break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (((val >> shift) & PUD_2711_MASK) != BCM2711_PULL_DOWN)
			return -EINVAL;

		*config = pinconf_to_config_packed(param, 50000);
		break;

	default:
		return bcm2835_pinconf_get(pctldev, pin, config);
	}

	return 0;
}

static void bcm2711_pull_config_set(struct bcm2835_pinctrl *pc,
				    unsigned int pin, unsigned int arg)
{
	u32 shifter;
	u32 value;
	u32 off;

	off = PUD_2711_REG_OFFSET(pin);
	shifter = PUD_2711_REG_SHIFT(pin);

	value = readl(pc->base + GP_GPIO_PUP_PDN_CNTRL_REG0 + (off * 4));
	value &= ~(PUD_2711_MASK << shifter);
	value |= (arg << shifter);
	writel(value, pc->base + GP_GPIO_PUP_PDN_CNTRL_REG0 + (off * 4));
}

static int bcm2711_pinconf_set(struct pinctrl_dev *pctldev,
			       unsigned int pin, unsigned long *configs,
			       unsigned int num_configs)
{
	struct bcm2835_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	u32 param, arg;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		/* convert legacy brcm,pull */
		case BCM2835_PINCONF_PARAM_PULL:
			if (arg == BCM2835_PUD_UP)
				arg = BCM2711_PULL_UP;
			else if (arg == BCM2835_PUD_DOWN)
				arg = BCM2711_PULL_DOWN;
			else
				arg = BCM2711_PULL_NONE;

			bcm2711_pull_config_set(pc, pin, arg);
			break;

		/* Set pull generic bindings */
		case PIN_CONFIG_BIAS_DISABLE:
			bcm2711_pull_config_set(pc, pin, BCM2711_PULL_NONE);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			bcm2711_pull_config_set(pc, pin, BCM2711_PULL_DOWN);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			bcm2711_pull_config_set(pc, pin, BCM2711_PULL_UP);
			break;

		/* Set output-high or output-low */
		case PIN_CONFIG_OUTPUT:
			bcm2835_gpio_set_bit(pc, arg ? GPSET0 : GPCLR0, pin);
			break;

		default:
			return -ENOTSUPP;
		}
	} /* for each config */

	return 0;
}

static const struct pinconf_ops bcm2711_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = bcm2711_pinconf_get,
	.pin_config_set = bcm2711_pinconf_set,
};

static const struct pinctrl_desc bcm2835_pinctrl_desc = {
	.name = MODULE_NAME,
	.pins = bcm2835_gpio_pins,
	.npins = BCM2835_NUM_GPIOS,
	.pctlops = &bcm2835_pctl_ops,
	.pmxops = &bcm2835_pmx_ops,
	.confops = &bcm2835_pinconf_ops,
	.owner = THIS_MODULE,
};

static const struct pinctrl_desc bcm2711_pinctrl_desc = {
	.name = "pinctrl-bcm2711",
	.pins = bcm2835_gpio_pins,
	.npins = BCM2711_NUM_GPIOS,
	.pctlops = &bcm2835_pctl_ops,
	.pmxops = &bcm2835_pmx_ops,
	.confops = &bcm2711_pinconf_ops,
	.owner = THIS_MODULE,
};

static const struct pinctrl_gpio_range bcm2835_pinctrl_gpio_range = {
	.name = MODULE_NAME,
	.npins = BCM2835_NUM_GPIOS,
};

static const struct pinctrl_gpio_range bcm2711_pinctrl_gpio_range = {
	.name = "pinctrl-bcm2711",
	.npins = BCM2711_NUM_GPIOS,
};

struct bcm_plat_data {
	const struct gpio_chip *gpio_chip;
	const struct pinctrl_desc *pctl_desc;
	const struct pinctrl_gpio_range *gpio_range;
};

static const struct bcm_plat_data bcm2835_plat_data = {
	.gpio_chip = &bcm2835_gpio_chip,
	.pctl_desc = &bcm2835_pinctrl_desc,
	.gpio_range = &bcm2835_pinctrl_gpio_range,
};

static const struct bcm_plat_data bcm2711_plat_data = {
	.gpio_chip = &bcm2711_gpio_chip,
	.pctl_desc = &bcm2711_pinctrl_desc,
	.gpio_range = &bcm2711_pinctrl_gpio_range,
};




static const struct acpi_device_id bcm2835_acpi_ids[] = {
	{ "BCM2845", (kernel_ulong_t)&bcm2835_plat_data },
	{ }
};
MODULE_DEVICE_TABLE(acpi, bcm2835_acpi_ids);


static void bcm2835_gpio_irq_handle_bank(struct bcm2835_pinctrl *pc,
	unsigned int bank, u32 mask)
{
unsigned long events;
unsigned offset;
unsigned gpio;

events = readl(pc->base + GPEDS0 + bank * 4);
events &= mask;
events &= pc->enabled_irq_map[bank];
for_each_set_bit(offset, &events, 32) {
gpio = (32 * bank) + offset;
generic_handle_domain_irq(pc->gpio_chip.irq.domain, gpio);
}
}

static void bcm2835_gpio_irq_handler(struct irq_desc *desc)
{
    struct gpio_chip *chip = irq_desc_get_handler_data(desc);
    struct bcm2835_pinctrl *pc = gpiochip_get_data(chip);
    struct irq_chip *host_chip = irq_desc_get_chip(desc);
    int irq = irq_desc_get_irq(desc);
    int group = 0;
    int i;

    for (i = 0; i < BCM2835_NUM_IRQS; i++) {
        if (chip->irq.parents[i] == irq) {
            group = i;
            break;
        }
    }
    BUG_ON(i == BCM2835_NUM_IRQS);

    chained_irq_enter(host_chip, desc);

    switch (group) {
    case 0: /* IRQ0 covers GPIOs 0-27 */
        bcm2835_gpio_irq_handle_bank(pc, 0, 0x0fffffff);
        break;
    case 1: /* IRQ1 covers GPIOs 28-45 */
        bcm2835_gpio_irq_handle_bank(pc, 0, 0xf0000000);
        bcm2835_gpio_irq_handle_bank(pc, 1, 0x00003fff);
        break;
    case 2: /* IRQ2 covers GPIOs 46-57 */
        bcm2835_gpio_irq_handle_bank(pc, 1, 0x003fc000);
        break;
    }

    chained_irq_exit(host_chip, desc);
}
static int bcm2835_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2835_pinctrl *pc;
	const struct acpi_device_id *id;
	const struct bcm_plat_data *pdata;
	struct resource *res;
	struct gpio_irq_chip *girq;
	struct resource irq_res;
	int ret, i;

	dev_info(dev, "BCM2835 pinctrl: probing via ACPI\n");

	id = acpi_match_device(bcm2835_acpi_ids, dev);
	if (!id) {
		dev_err(dev, "No matching ACPI ID\n");
		return -ENODEV;
	}
	pdata = (const struct bcm_plat_data *)id->driver_data;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	platform_set_drvdata(pdev, pc);
	pc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Missing memory resource\n");
		return -ENODEV;
	}

	pc->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pc->base)) {
		dev_err(dev, "Failed to map registers\n");
		return PTR_ERR(pc->base);
	}

	dev_info(dev, "Mapped GPIO registers at %pa\n", &res->start);

	spin_lock_init(&pc->fsel_lock);

	for (i = 0; i < BCM2835_NUM_BANKS; i++) {
		unsigned long events;
		unsigned offset;
		void __iomem *base = pc->base;
		void __iomem *reg;

		// Clear event detection registers
		writel(0, base + GPREN0  + i * 4);
		writel(0, base + GPFEN0  + i * 4);
		writel(0, base + GPHEN0  + i * 4);
		writel(0, base + GPLEN0  + i * 4);
		writel(0, base + GPAREN0 + i * 4);
		writel(0, base + GPAFEN0 + i * 4);

		// Clear events
		reg = base + GPEDS0 + i * 4;
		events = readl(reg);
		for_each_set_bit(offset, &events, 32)
			writel(BIT(offset), reg);

		raw_spin_lock_init(&pc->irq_lock[i]);
	}

	pc->gpio_chip = *pdata->gpio_chip;
	pc->gpio_chip.parent = dev;
	pc->gpio_chip.base = -1;

	girq = &pc->gpio_chip.irq;
	gpio_irq_chip_set_chip(girq, &bcm2835_gpio_irq_chip);
	girq->parent_handler = bcm2835_gpio_irq_handler;
	girq->num_parents = BCM2835_NUM_IRQS;
	girq->parents = devm_kcalloc(dev, BCM2835_NUM_IRQS, sizeof(*girq->parents), GFP_KERNEL);
	if (!girq->parents) {
		return -ENOMEM;
	}


		for (i = 0; i < BCM2835_NUM_IRQS; i++) {
			int ret = acpi_irq_get(ACPI_HANDLE(dev), i, &irq_res);
			if (ret < 0)
				break;
		
			girq->parents[i] = irq_res.start;
		}
		girq->num_parents = i;

	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_level_irq;

	pc->wake_irq = devm_kcalloc(dev, BCM2835_NUM_IRQS, sizeof(*pc->wake_irq), GFP_KERNEL);
	if (!pc->wake_irq)
		return -ENOMEM;

	for (i = 0; i < BCM2835_NUM_IRQS; i++) {
		char *name;
		int irq, len;

		irq = platform_get_irq(pdev, i + BCM2835_NUM_IRQS + 1);
		pc->wake_irq[i] = irq;
		if (irq < 0)
			continue;

		len = strlen(dev_name(dev)) + 16;
		name = devm_kzalloc(dev, len, GFP_KERNEL);
		if (!name)
			return -ENOMEM;

		snprintf(name, len, "%s:bank%d", dev_name(dev), i);
		ret = devm_request_irq(dev, irq,
				       bcm2835_gpio_wake_irq_handler,
				       IRQF_SHARED, name, pc);
		if (ret)
			dev_warn(dev, "Failed to request wake IRQ %d\n", irq);
	}

	pc->pctl_desc = *pdata->pctl_desc;
	pc->pctl_desc.owner = THIS_MODULE;
	dev_set_name(dev, "BCM2841:00");


	pc->pctl_dev = devm_pinctrl_register(dev, &pc->pctl_desc, pc);
	if (IS_ERR(pc->pctl_dev)) {
		dev_err(dev, "Failed to register pinctrl device\n");
		return PTR_ERR(pc->pctl_dev);
	}


	pc->gpio_range = *pdata->gpio_range;
	pc->gpio_range.base = 0;
	pc->gpio_range.gc = &pc->gpio_chip;

	ret = devm_gpiochip_add_data(dev, &pc->gpio_chip, pc);
	if (ret) {
		dev_err(dev, "Failed to register GPIO chip: %d\n", ret);
		goto cleanup_gpio_range;
	}

	dev_info(dev, "Registering pin range for devname: %s\n", pinctrl_dev_get_devname(pc->pctl_dev));

	ret = gpiochip_add_pin_range(&pc->gpio_chip,
				     pinctrl_dev_get_devname(pc->pctl_dev),
				     0, 0, pc->gpio_chip.ngpio);
	if (ret) {
		dev_err(dev, "Failed to add GPIO pin range: %d\n", ret);
		goto cleanup_gpio_range;
	}
    pc->gpio_range.pin_base=0;
	pinctrl_add_gpio_range(pc->pctl_dev, &pc->gpio_range);

	pc->i2c_range = (struct pinctrl_gpio_range) {
		.gc = &pc->gpio_chip,
		.name = "BCM2841:00",
		.base = 2,
		.pin_base = 2,
		.npins = 2,
		.id = 0,
	};
	
	pinctrl_add_gpio_range(pc->pctl_dev, &pc->i2c_range);
	
	dev_info(dev, "Added pinctrl range for I2C device BCM2841:00 (gpio2-3)\n");

	dev_info(dev, "pinctrl dev: %s", dev_name(dev));
	dev_info(dev, "pinctrl dev->fwnode: %p\n", dev->fwnode);
	

	dev_info(dev, "BCM GPIO controller registered successfully (ngpio=%d)\n",
		 pc->gpio_chip.ngpio);




	return 0;

cleanup_gpio_range:
	pinctrl_remove_gpio_range(pc->pctl_dev, &pc->gpio_range);
	return ret;
}

static struct platform_driver bcm2835_pinctrl_driver = {
	.probe = bcm2835_pinctrl_probe,
	.driver = {
		.name = "pinctrl-bcm2835-acpi",
		.acpi_match_table = ACPI_PTR(bcm2835_acpi_ids),
	},
};
module_platform_driver(bcm2835_pinctrl_driver);

MODULE_AUTHOR("Chris Boot");
MODULE_AUTHOR("Simon Arlott");
MODULE_AUTHOR("Stephen Warren");
MODULE_DESCRIPTION("Broadcom BCM2835/2711 pinctrl and GPIO driver modified for ACPI");
MODULE_LICENSE("GPL");
