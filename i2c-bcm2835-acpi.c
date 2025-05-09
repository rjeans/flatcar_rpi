// SPDX-License-Identifier: GPL-2.0
/*
 * BCM2835 I2C controller driver
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h> 
#include <linux/pinctrl/machine.h>


#define BCM2835_I2C_C		0x0
#define BCM2835_I2C_S		0x4
#define BCM2835_I2C_DLEN	0x8
#define BCM2835_I2C_A		0xc
#define BCM2835_I2C_FIFO	0x10
#define BCM2835_I2C_DIV		0x14
#define BCM2835_I2C_DEL		0x18
/*
 * 16-bit field for the number of SCL cycles to wait after rising SCL
 * before deciding the target is not responding. 0 disables the
 * timeout detection.
 */
#define BCM2835_I2C_CLKT	0x1c

#define BCM2835_I2C_C_READ	BIT(0)
#define BCM2835_I2C_C_CLEAR	BIT(4) /* bits 4 and 5 both clear */
#define BCM2835_I2C_C_ST	BIT(7)
#define BCM2835_I2C_C_INTD	BIT(8)
#define BCM2835_I2C_C_INTT	BIT(9)
#define BCM2835_I2C_C_INTR	BIT(10)
#define BCM2835_I2C_C_I2CEN	BIT(15)

#define BCM2835_I2C_S_TA	BIT(0)
#define BCM2835_I2C_S_DONE	BIT(1)
#define BCM2835_I2C_S_TXW	BIT(2)
#define BCM2835_I2C_S_RXR	BIT(3)
#define BCM2835_I2C_S_TXD	BIT(4)
#define BCM2835_I2C_S_RXD	BIT(5)
#define BCM2835_I2C_S_TXE	BIT(6)
#define BCM2835_I2C_S_RXF	BIT(7)
#define BCM2835_I2C_S_ERR	BIT(8)
#define BCM2835_I2C_S_CLKT	BIT(9)
#define BCM2835_I2C_S_LEN	BIT(10) /* Fake bit for SW error reporting */

#define BCM2835_I2C_FEDL_SHIFT	16
#define BCM2835_I2C_REDL_SHIFT	0

#define BCM2835_I2C_CDIV_MIN	0x0002
#define BCM2835_I2C_CDIV_MAX	0xFFFE

static const struct pinctrl_map bcm2835_i2c1_map[] = {
    {
        .dev_name = "BCM2841:00",
        .name = "default",
        .type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "BCM2845:00",
        .data.mux = {
            .group = "gpio2",
            .function = "alt0",
        },
    },
    {
        .dev_name = "BCM2841:00",
        .name = "default",
        .type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "BCM2845:00",
        .data.mux = {
            .group = "gpio3",
            .function = "alt0",
        },
    },
};


struct bcm2835_i2c_dev {
	struct device *dev;
	void __iomem *regs;
	int irq;
	struct i2c_adapter adapter;
	struct completion completion;
	struct i2c_msg *curr_msg;
	struct clk *bus_clk;
	int num_msgs;
	u32 msg_err;
	u8 *msg_buf;
	size_t msg_buf_remaining;
};

static inline void bcm2835_i2c_writel(struct bcm2835_i2c_dev *i2c_dev,
				      u32 reg, u32 val)
{
	writel(val, i2c_dev->regs + reg);
}

static inline u32 bcm2835_i2c_readl(struct bcm2835_i2c_dev *i2c_dev, u32 reg)
{
	return readl(i2c_dev->regs + reg);
}

#define to_clk_bcm2835_i2c(_hw) container_of(_hw, struct clk_bcm2835_i2c, hw)
struct clk_bcm2835_i2c {
	struct clk_hw hw;
	struct bcm2835_i2c_dev *i2c_dev;
};

static int clk_bcm2835_i2c_calc_divider(unsigned long rate,
				unsigned long parent_rate)
{
	u32 divider = DIV_ROUND_UP(parent_rate, rate);

	/*
	 * Per the datasheet, the register is always interpreted as an even
	 * number, by rounding down. In other words, the LSB is ignored. So,
	 * if the LSB is set, increment the divider to avoid any issue.
	 */
	if (divider & 1)
		divider++;
	if ((divider < BCM2835_I2C_CDIV_MIN) ||
	    (divider > BCM2835_I2C_CDIV_MAX))
		return -EINVAL;

	return divider;
}

static int clk_bcm2835_i2c_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_bcm2835_i2c *div = to_clk_bcm2835_i2c(hw);
	u32 redl, fedl;
	u32 divider = clk_bcm2835_i2c_calc_divider(rate, parent_rate);

	if (divider == -EINVAL)
		return -EINVAL;

	bcm2835_i2c_writel(div->i2c_dev, BCM2835_I2C_DIV, divider);

	/*
	 * Number of core clocks to wait after falling edge before
	 * outputting the next data bit.  Note that both FEDL and REDL
	 * can't be greater than CDIV/2.
	 */
	fedl = max(divider / 16, 1u);

	/*
	 * Number of core clocks to wait after rising edge before
	 * sampling the next incoming data bit.
	 */
	redl = max(divider / 4, 1u);

	bcm2835_i2c_writel(div->i2c_dev, BCM2835_I2C_DEL,
			   (fedl << BCM2835_I2C_FEDL_SHIFT) |
			   (redl << BCM2835_I2C_REDL_SHIFT));
	return 0;
}

static long clk_bcm2835_i2c_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	u32 divider = clk_bcm2835_i2c_calc_divider(rate, *parent_rate);

	return DIV_ROUND_UP(*parent_rate, divider);
}

static unsigned long clk_bcm2835_i2c_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct clk_bcm2835_i2c *div = to_clk_bcm2835_i2c(hw);
	u32 divider = bcm2835_i2c_readl(div->i2c_dev, BCM2835_I2C_DIV);

	return DIV_ROUND_UP(parent_rate, divider);
}

static const struct clk_ops clk_bcm2835_i2c_ops = {
	.set_rate = clk_bcm2835_i2c_set_rate,
	.round_rate = clk_bcm2835_i2c_round_rate,
	.recalc_rate = clk_bcm2835_i2c_recalc_rate,
};


static void bcm2835_fill_txfifo(struct bcm2835_i2c_dev *i2c_dev)
{
	u32 val;

	while (i2c_dev->msg_buf_remaining) {
		val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
		if (!(val & BCM2835_I2C_S_TXD))
			break;
		bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_FIFO,
				   *i2c_dev->msg_buf);
		i2c_dev->msg_buf++;
		i2c_dev->msg_buf_remaining--;
	}
}

static void bcm2835_drain_rxfifo(struct bcm2835_i2c_dev *i2c_dev)
{
	u32 val;

	while (i2c_dev->msg_buf_remaining) {
		val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
		if (!(val & BCM2835_I2C_S_RXD))
			break;
		*i2c_dev->msg_buf = bcm2835_i2c_readl(i2c_dev,
						      BCM2835_I2C_FIFO);
		i2c_dev->msg_buf++;
		i2c_dev->msg_buf_remaining--;
	}
}

/*
 * Repeated Start Condition (Sr)
 * The BCM2835 ARM Peripherals datasheet mentions a way to trigger a Sr when it
 * talks about reading from a target with 10 bit address. This is achieved by
 * issuing a write, poll the I2CS.TA flag and wait for it to be set, and then
 * issue a read.
 * A comment in https://github.com/raspberrypi/linux/issues/254 shows how the
 * firmware actually does it using polling and says that it's a workaround for
 * a problem in the state machine.
 * It turns out that it is possible to use the TXW interrupt to know when the
 * transfer is active, provided the FIFO has not been prefilled.
 */

static void bcm2835_i2c_start_transfer(struct bcm2835_i2c_dev *i2c_dev)
{
	u32 c = BCM2835_I2C_C_ST | BCM2835_I2C_C_I2CEN;
	struct i2c_msg *msg = i2c_dev->curr_msg;
	bool last_msg = (i2c_dev->num_msgs == 1);

	if (!i2c_dev->num_msgs)
		return;

	i2c_dev->num_msgs--;
	i2c_dev->msg_buf = msg->buf;
	i2c_dev->msg_buf_remaining = msg->len;

	if (msg->flags & I2C_M_RD)
		c |= BCM2835_I2C_C_READ | BCM2835_I2C_C_INTR;
	else
		c |= BCM2835_I2C_C_INTT;

	if (last_msg)
		c |= BCM2835_I2C_C_INTD;

	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_A, msg->addr);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_DLEN, msg->len);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, c);
}

static void bcm2835_i2c_finish_transfer(struct bcm2835_i2c_dev *i2c_dev)
{
	i2c_dev->curr_msg = NULL;
	i2c_dev->num_msgs = 0;

	i2c_dev->msg_buf = NULL;
	i2c_dev->msg_buf_remaining = 0;
}

/*
 * Note about I2C_C_CLEAR on error:
 * The I2C_C_CLEAR on errors will take some time to resolve -- if you were in
 * non-idle state and I2C_C_READ, it sets an abort_rx flag and runs through
 * the state machine to send a NACK and a STOP. Since we're setting CLEAR
 * without I2CEN, that NACK will be hanging around queued up for next time
 * we start the engine.
 */

static irqreturn_t bcm2835_i2c_isr(int this_irq, void *data)
{
	struct bcm2835_i2c_dev *i2c_dev = data;
	u32 val, err;

	val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);

	// If no meaningful bits are set, return IRQ_NONE to avoid floods
if (!(val & (BCM2835_I2C_S_DONE | BCM2835_I2C_S_ERR | BCM2835_I2C_S_CLKT))) {
	dev_warn(i2c_dev->dev, "Ignoring spurious IRQ (status=0x%08x)\n", val);
	return IRQ_NONE;

}

	err = val & (BCM2835_I2C_S_CLKT | BCM2835_I2C_S_ERR);
	if (err) {
		i2c_dev->msg_err = err;
		goto complete;
	}

	if (val & BCM2835_I2C_S_DONE) {
		if (!i2c_dev->curr_msg) {
			dev_err(i2c_dev->dev, "Got unexpected interrupt (from firmware?)\n");
		} else if (i2c_dev->curr_msg->flags & I2C_M_RD) {
			bcm2835_drain_rxfifo(i2c_dev);
			val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
		}

		if ((val & BCM2835_I2C_S_RXD) || i2c_dev->msg_buf_remaining)
			i2c_dev->msg_err = BCM2835_I2C_S_LEN;
		else
			i2c_dev->msg_err = 0;
		goto complete;
	}

	if (val & BCM2835_I2C_S_TXW) {
		if (!i2c_dev->msg_buf_remaining) {
			i2c_dev->msg_err = val | BCM2835_I2C_S_LEN;
			goto complete;
		}

		bcm2835_fill_txfifo(i2c_dev);

		if (i2c_dev->num_msgs && !i2c_dev->msg_buf_remaining) {
			i2c_dev->curr_msg++;
			bcm2835_i2c_start_transfer(i2c_dev);
		}

		return IRQ_HANDLED;
	}

	if (val & BCM2835_I2C_S_RXR) {
		if (!i2c_dev->msg_buf_remaining) {
			i2c_dev->msg_err = val | BCM2835_I2C_S_LEN;
			goto complete;
		}

		bcm2835_drain_rxfifo(i2c_dev);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;

complete:
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_S, BCM2835_I2C_S_CLKT |
			   BCM2835_I2C_S_ERR | BCM2835_I2C_S_DONE);
	complete(&i2c_dev->completion);

	return IRQ_HANDLED;
}

static int bcm2835_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			    int num)
{
	struct bcm2835_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	unsigned long time_left;
	int i;

	for (i = 0; i < (num - 1); i++)
		if (msgs[i].flags & I2C_M_RD) {
			dev_warn_once(i2c_dev->dev,
				      "only one read message supported, has to be last\n");
			return -EOPNOTSUPP;
		}

	i2c_dev->curr_msg = msgs;
	i2c_dev->num_msgs = num;
	reinit_completion(&i2c_dev->completion);

	bcm2835_i2c_start_transfer(i2c_dev);

	time_left = wait_for_completion_timeout(&i2c_dev->completion,
						adap->timeout);

	bcm2835_i2c_finish_transfer(i2c_dev);

	if (!time_left) {
		bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C,
				   BCM2835_I2C_C_CLEAR);
		return -ETIMEDOUT;
	}

	if (!i2c_dev->msg_err)
		return num;

	dev_dbg(i2c_dev->dev, "i2c transfer failed: %x\n", i2c_dev->msg_err);

	if (i2c_dev->msg_err & BCM2835_I2C_S_ERR)
		return -EREMOTEIO;

	return -EIO;
}

static u32 bcm2835_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm bcm2835_i2c_algo = {
	.master_xfer = bcm2835_i2c_xfer, // Corrected member name
	.functionality = bcm2835_i2c_func,
};

/*
 * The BCM2835 was reported to have problems with clock stretching:
 * https://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html
 * https://www.raspberrypi.org/forums/viewtopic.php?p=146272
 */
static const struct i2c_adapter_quirks bcm2835_i2c_quirks = {
	.flags = I2C_AQ_NO_CLK_STRETCH,
};

static const struct acpi_device_id bcm2835_i2c_acpi_match[] = {
    { "BCM2841", 0},
    {},
};
MODULE_DEVICE_TABLE(acpi, bcm2835_i2c_acpi_match);

static int bcm2835_i2c_probe(struct platform_device *pdev)
{
	struct bcm2835_i2c_dev *i2c_dev;
	int i,ret;
	struct i2c_adapter *adap;
	struct clk *mclk;
	u32 bus_clk_rate;
	unsigned long long uid;
	struct pinctrl *pinctrl;

	// Check for ACPI _UID
	if (ACPI_HANDLE(&pdev->dev)) {
		ret = acpi_evaluate_integer(ACPI_HANDLE(&pdev->dev), "_UID", NULL, &uid);
		if (ret) {
			dev_err(&pdev->dev, "Failed to evaluate _UID\n");
			return -EINVAL;
		}
		if (uid != 0x1) {
			dev_info(&pdev->dev, "Skipping device with _UID 0x%llx\n", uid); // Corrected format specifier
			return -ENODEV;
		}
	}

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2c_dev);
	i2c_dev->dev = &pdev->dev;

	init_completion(&i2c_dev->completion);

	i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);

	mclk = NULL;
	dev_info(&pdev->dev, "No ACPI clock provided; using fallback core clock rate\n");
		
	unsigned long core_clk_hz = 250000000; // fallback core clock, 250 MHz
	u32 divider, fedl, redl;

	if (!bus_clk_rate) {
		dev_warn(&pdev->dev,
			"clock-frequency is zero or missing; defaulting to 100000 Hz\n");
		bus_clk_rate = 100000;
	}
	
	divider = DIV_ROUND_UP(core_clk_hz, bus_clk_rate);
	if (divider & 1)
		divider++; // ensure even value
	
	if (divider < BCM2835_I2C_CDIV_MIN || divider > BCM2835_I2C_CDIV_MAX) {
		dev_err(&pdev->dev, "Invalid clock divider: %u\n", divider);
		return -EINVAL;
	}
	
	fedl = max(divider / 16, 1u);
	redl = max(divider / 4, 1u);
	
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_DIV, divider);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_DEL,
		(fedl << BCM2835_I2C_FEDL_SHIFT) | (redl << BCM2835_I2C_REDL_SHIFT));
	
	dev_info(&pdev->dev, "Fallback clock set directly: divider=%u, fedl=%u, redl=%u\n",
		divider, fedl, redl);
	

	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0) {
		ret = i2c_dev->irq;
		goto err_disable_unprepare_clk;
	}

	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, 0);  // Disable all interrupts + controller


	ret = request_irq(i2c_dev->irq, bcm2835_i2c_isr, IRQF_SHARED,
			  dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto err_disable_unprepare_clk;
	}

	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
    bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_S,
    BCM2835_I2C_S_CLKT | BCM2835_I2C_S_ERR | BCM2835_I2C_S_DONE |
    BCM2835_I2C_S_TXW | BCM2835_I2C_S_RXR);

	dev_info(i2c_dev->dev, "Cleared pending IRQ conditions: status=0x%08x\n",
		bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S));

		adap = &i2c_dev->adapter;

		/* Link driver data */
		i2c_set_adapdata(adap, i2c_dev);
		adap->owner = THIS_MODULE;
		adap->class = I2C_CLASS_DEPRECATED;
	
		/* Adapter name — safe for both DT and ACPI */
		snprintf(adap->name, sizeof(adap->name), "bcm2835 (%s)",
				 dev_name(&pdev->dev) ? dev_name(&pdev->dev) : "unknown");
	
		adap->algo = &bcm2835_i2c_algo;
	
		/* Parent device is the platform device */
		adap->dev.parent = &pdev->dev;
	
		/* Only assign .fwnode if it's valid (safe for ACPI) */
		{
			struct fwnode_handle *fwnode = dev_fwnode(&pdev->dev);
			if (fwnode) {
				adap->dev.fwnode = fwnode;
				dev_info(&pdev->dev, "Linked firmware node: %pfw\n", fwnode);
			} else {
				dev_warn(&pdev->dev, "No firmware node found; adapter will have no fwnode\n");
			}
		}
	
		/* Set I2C controller quirks */
		adap->quirks = &bcm2835_i2c_quirks;
	

	/*
	 * Disable the hardware clock stretching timeout. SMBUS
	 * specifies a limit for how long the device can stretch the
	 * clock, but core I2C doesn't.
	 */
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_CLKT, 0);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, 0);

    for (i=0; i<ARRAY_SIZE(bcm2835_i2c1_map); i++) {
		dev_info(&pdev->dev, "bcm2835_i2c1_map[%d].dev_name: %s\n", i, bcm2835_i2c1_map[i].dev_name);
		dev_info(&pdev->dev, "bcm2835_i2c1_map[%d].name: %s\n", i, bcm2835_i2c1_map[i].name);
		dev_info(&pdev->dev, "bcm2835_i2c1_map[%d].type: %d\n", i, bcm2835_i2c1_map[i].type);
	}

	ret = pinctrl_register_mappings(bcm2835_i2c1_map, ARRAY_SIZE(bcm2835_i2c1_map));
	if (ret)
		dev_warn(&pdev->dev, "Failed to register pinctrl mappings: %d\n", ret);

	dev_info(&pdev->dev, "Before pinctrl_get_select_default\n");


	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	dev_info(&pdev->dev, "After pinctrl_get_select_default: pinctrl=%p, ERR=%d\n",
		pinctrl, PTR_ERR_OR_ZERO(pinctrl));


	if (IS_ERR(pinctrl)) {
		dev_warn(&pdev->dev, "Failed to apply default pinctrl state\n");
	} else {
		dev_info(&pdev->dev, "Applied default pinctrl state\n");
	}
	struct fwnode_handle *child;

	fwnode_for_each_child_node(dev_fwnode(&pdev->dev), child) {
		if (!is_acpi_node(child)) {
			dev_warn(&pdev->dev, "Skipping non-ACPI child fwnode\n");
			continue;
		}
	
		u64 adr;
		if (fwnode_property_read_u64(child, "reg", &adr)) {
			dev_warn(&pdev->dev, "Child has no reg property\n");
			continue;
		}
	
		dev_info(&pdev->dev, "ACPI child with reg=0x%llx\n", adr);
	}
	

	ret = i2c_add_adapter(adap);
	if (ret)
		goto err_free_irq;

	

	dev_info(&pdev->dev, "dev_name: %s", dev_name(&pdev->dev));
    dev_info(&pdev->dev, "dev->fwnode: %p\n", pdev->dev.fwnode);
	dev_info(&pdev->dev, "fwnode: %p\n", dev_fwnode(&pdev->dev));
	dev_info(&pdev->dev, "pinctrl map dev_name: %s\n", bcm2835_i2c1_map[0].dev_name);
    dev_info(&pdev->dev, "actual dev_name: %s\n", dev_name(&pdev->dev));

	

	return 0;

err_free_irq:
	free_irq(i2c_dev->irq, i2c_dev);
err_disable_unprepare_clk:
	clk_disable_unprepare(i2c_dev->bus_clk);


	return ret;
}

static int bcm2835_i2c_remove(struct platform_device *pdev)
{
	struct bcm2835_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	clk_rate_exclusive_put(i2c_dev->bus_clk);
	clk_disable_unprepare(i2c_dev->bus_clk);

	free_irq(i2c_dev->irq, i2c_dev);
	i2c_del_adapter(&i2c_dev->adapter);

	return 0; // Return success
}

static struct platform_driver bcm2835_i2c_driver = {
	.probe = bcm2835_i2c_probe,
	.remove = bcm2835_i2c_remove,
	.driver = {
		.name = "i2c-bcm2835",
		.acpi_match_table = bcm2835_i2c_acpi_match,
	},
};
module_platform_driver(bcm2835_i2c_driver);

MODULE_AUTHOR("Stephen Warren <swarren@wwwdotorg.org>");
MODULE_DESCRIPTION("BCM2835 I2C bus adapter");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-bcm2835");
