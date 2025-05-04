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
#include <linux/property.h>

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

#define DEFAULT_I2C_FREQ 100000


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

static struct clk *bcm2835_i2c_register_div(struct device *dev,
					struct clk *mclk,
					struct bcm2835_i2c_dev *i2c_dev)
{
	struct clk_init_data init;
	struct clk_bcm2835_i2c *priv;
	char name[32];
	const char *mclk_name;

	snprintf(name, sizeof(name), "%s_div", dev_name(dev));

	// Use a default parent clock name if mclk is NULL
	mclk_name = mclk ? __clk_get_name(mclk) : "default_parent_clk";

	init.ops = &clk_bcm2835_i2c_ops;
	init.name = name;
	init.parent_names = (const char* []) { mclk_name };
	init.num_parents = 1;
	init.flags = 0;

	priv = devm_kzalloc(dev, sizeof(struct clk_bcm2835_i2c), GFP_KERNEL);
	if (priv == NULL)
		return ERR_PTR(-ENOMEM);

	priv->hw.init = &init;
	priv->i2c_dev = i2c_dev;

	clk_hw_register_clkdev(&priv->hw, "div", dev_name(dev));
	return devm_clk_register(dev, &priv->hw);
}

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

static void bcm2835_i2c_dump_regs(struct bcm2835_i2c_dev *i2c_dev)
{
	dev_err(i2c_dev->dev, "Register dump:\n");
	dev_err(i2c_dev->dev, "  BCM2835_I2C_C:    0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_C));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_S:    0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_DLEN: 0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DLEN));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_A:    0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_A));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_FIFO: 0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_FIFO));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_DIV:  0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DIV));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_DEL:  0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DEL));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_CLKT: 0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_CLKT));
}

static irqreturn_t bcm2835_i2c_isr(int this_irq, void *data)
{
	struct bcm2835_i2c_dev *i2c_dev = data;
	u32 val, err;
	static int spurious_count = 0;

	// Read the interrupt status
	val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
	if (val == 0) {
		// Spurious interrupt, increment counter
		spurious_count++;
		dev_warn(i2c_dev->dev, "Spurious IRQ, status: 0x%x (count: %d)\n", val, spurious_count);

		 // Dump registers for debugging
		bcm2835_i2c_dump_regs(i2c_dev);

		// Disable IRQ if spurious interrupts exceed threshold
		if (spurious_count > 10) {
			dev_err(i2c_dev->dev, "Disabling IRQ due to repeated spurious interrupts\n");
			disable_irq_nosync(i2c_dev->irq);
		}
		return IRQ_NONE;
	}

	// Reset spurious interrupt counter
	spurious_count = 0;

	dev_dbg(i2c_dev->dev, "ISR triggered, status: 0x%x\n", val);

	// Check for errors
	err = val & (BCM2835_I2C_S_CLKT | BCM2835_I2C_S_ERR);
	if (err) {
		dev_err(i2c_dev->dev, "ISR error: 0x%x\n", err);
		i2c_dev->msg_err = err;
		goto complete;
	}

	// Check for transfer complete
	if (val & BCM2835_I2C_S_DONE) {
		dev_dbg(i2c_dev->dev, "ISR transfer complete\n");
		if (!i2c_dev->curr_msg) {
			dev_err(i2c_dev->dev, "Unexpected interrupt (no current message)\n");
		} else if (i2c_dev->curr_msg->flags & I2C_M_RD) {
			bcm2835_drain_rxfifo(i2c_dev);
		} else {
			i2c_dev->msg_err = 0;
		}
		goto complete;
	}

	// Handle TX FIFO empty
	if (val & BCM2835_I2C_S_TXW) {
		dev_dbg(i2c_dev->dev, "ISR TX FIFO empty\n");
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

	// Handle RX FIFO full
	if (val & BCM2835_I2C_S_RXR) {
		dev_dbg(i2c_dev->dev, "ISR RX FIFO full\n");
		if (!i2c_dev->msg_buf_remaining) {
			i2c_dev->msg_err = val | BCM2835_I2C_S_LEN;
			goto complete;
		}
		bcm2835_drain_rxfifo(i2c_dev);
		return IRQ_HANDLED;
	}

	// If no conditions are met, return IRQ_NONE
	dev_warn(i2c_dev->dev, "Unhandled IRQ, status: 0x%x\n", val);
	return IRQ_NONE;

complete:
	// Clear the interrupt and signal completion
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
	.master_xfer = bcm2835_i2c_xfer,
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

static int bcm2835_i2c_probe(struct platform_device *pdev)
{
	struct bcm2835_i2c_dev *i2c_dev;
	struct i2c_adapter *adap;
	struct clk *mclk;
	u32 bus_clk_rate = DEFAULT_I2C_FREQ; // Default: 100kHz
	bool irq_pending;
	int ret;
	


	/* Allocate and initialize private data */
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->dev = &pdev->dev;
	init_completion(&i2c_dev->completion);
	platform_set_drvdata(pdev, i2c_dev);

	/* Map I/O memory */
	i2c_dev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);

	/* Try to get ACPI clock-frequency override */
	if (device_property_read_u32(&pdev->dev, "clock-frequency", &bus_clk_rate)) {
		bus_clk_rate = DEFAULT_I2C_FREQ;
		dev_warn(&pdev->dev, "clock-frequency not specified, defaulting to %u Hz\n", bus_clk_rate);
	}
	

	/* Get parent clock */
	mclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(mclk)) {
		dev_warn(&pdev->dev, "Parent clock not found, using fallback rate\n");
		mclk = NULL;
		
	} else {
		dev_info(&pdev->dev, "Parent clock rate: %lu Hz\n", clk_get_rate(mclk));
	}

	if (!mclk) {
		u32 divider = clk_bcm2835_i2c_calc_divider(bus_clk_rate, DEFAULT_I2C_FREQ); 
	
		if (divider == -EINVAL) {
			dev_err(&pdev->dev, "Invalid fallback divider for clock-frequency %u\n", bus_clk_rate);
			return -EINVAL;
		}
	
		/* Write clock divider and delays manually */
		bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_DIV, divider);
	
		u32 fedl = max(divider / 16, 1u);
		u32 redl = max(divider / 4, 1u);
		bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_DEL,
						   (fedl << BCM2835_I2C_FEDL_SHIFT) |
						   (redl << BCM2835_I2C_REDL_SHIFT));
		
		bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, BCM2835_I2C_C_I2CEN);

		dev_info(&pdev->dev, "Fallback clock set directly: divider=%u, fedl=%u, redl=%u\n",
				 divider, fedl, redl);
		dev_info(&pdev->dev, "Final I2C_DIV: 0x%08x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DIV));
		dev_info(&pdev->dev, "Final I2C_DEL: 0x%08x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DEL));
				 
	} else {
	

	/* Register I2C bus clock divider */
	i2c_dev->bus_clk = bcm2835_i2c_register_div(&pdev->dev, mclk, i2c_dev);
	if (IS_ERR(i2c_dev->bus_clk)) {
		ret = PTR_ERR(i2c_dev->bus_clk);
		dev_err(&pdev->dev, "Failed to register bus clock: %d\n", ret);
		goto err_put_exclusive_rate;
	}


	/* Set and enable the clock */
	if (i2c_dev->bus_clk) {
		ret = clk_set_rate_exclusive(i2c_dev->bus_clk, bus_clk_rate);
		if (ret < 0)
			dev_warn(&pdev->dev, "Could not set clock rate: %d (continuing)\n", ret);
	
		ret = clk_prepare_enable(i2c_dev->bus_clk);
		if (ret)
			dev_warn(&pdev->dev, "Could not enable clock: %d (continuing)\n", ret);
	
		dev_info(&pdev->dev, "Using bus clock rate: %lu Hz\n", clk_get_rate(i2c_dev->bus_clk));
	} else {
		dev_info(&pdev->dev, "No clock provided, using hardcoded fallback rate: %u Hz\n", bus_clk_rate);
	}
	
}
	/* Acquire and register IRQ */
	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0) {
		ret = i2c_dev->irq;
		goto err_disable_unprepare_clk;
	}

	dev_info(&pdev->dev, "IRQ %d registered for BCM2835 I2C\n", i2c_dev->irq);

	/* Check if IRQ is pending at startup */
	if (irq_get_irqchip_state(i2c_dev->irq, IRQCHIP_STATE_PENDING, &irq_pending)) {
		dev_warn(&pdev->dev, "Failed to query IRQ %d state\n", i2c_dev->irq);
	} else if (irq_pending) {
		dev_warn(&pdev->dev, "IRQ %d is pending at probe\n", i2c_dev->irq);
	}

	ret = request_irq(i2c_dev->irq, bcm2835_i2c_isr,
	                  IRQF_SHARED | IRQF_NO_SUSPEND,
	                  dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		goto err_disable_unprepare_clk;
	}

	/* Set up the I2C adapter */
	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	adap->algo = &bcm2835_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->quirks = &bcm2835_i2c_quirks;
	snprintf(adap->name, sizeof(adap->name), "bcm2835 (%s)", dev_name(&pdev->dev));

	/* Disable clock stretching timeout */
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_CLKT, 0);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, 0);

	ret = i2c_add_adapter(adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add I2C adapter\n");
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	free_irq(i2c_dev->irq, i2c_dev);
err_disable_unprepare_clk:
	if (i2c_dev->bus_clk)
		clk_disable_unprepare(i2c_dev->bus_clk);
	else
		dev_warn(&pdev->dev, "No clock to disable\n");
	
err_put_exclusive_rate:
	if (i2c_dev->bus_clk)
		clk_rate_exclusive_put(i2c_dev->bus_clk);
	else
		dev_warn(&pdev->dev, "No clock to put exclusive rate\n");
	return ret;
}

static int bcm2835_i2c_remove(struct platform_device *pdev)
{
	struct bcm2835_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	if (i2c_dev->bus_clk) {
		clk_disable_unprepare(i2c_dev->bus_clk);
		clk_rate_exclusive_put(i2c_dev->bus_clk);
	}

	free_irq(i2c_dev->irq, i2c_dev);
	i2c_del_adapter(&i2c_dev->adapter);

	return 0;
}

static const struct acpi_device_id bcm2835_i2c_acpi_match[] = {
	{ "BCM2835", 0 },
	{ "BCM2711", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, bcm2835_i2c_acpi_match);

static struct platform_driver bcm2835_i2c_driver = {
	.probe		= bcm2835_i2c_probe,
	.remove		= bcm2835_i2c_remove,
	.driver		= {
		.name	= "i2c-bcm2835",
		.acpi_match_table = ACPI_PTR(bcm2835_i2c_acpi_match),
	},
};
module_platform_driver(bcm2835_i2c_driver);

MODULE_AUTHOR("Stephen Warren <swarren@wwwdotorg.org>");
MODULE_DESCRIPTION("BCM2835 I2C bus adapter");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-bcm2835");
