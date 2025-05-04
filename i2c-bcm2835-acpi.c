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

static irqreturn_t bcm2835_i2c_isr(int this_irq, void *data)c_dev)
{
	struct bcm2835_i2c_dev *i2c_dev = data;");
	u32 val, err;ev->dev, "  BCM2835_I2C_C:    0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_C));
	static int spurious_count = 0;35_I2C_S:    0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S));
	dev_err(i2c_dev->dev, "  BCM2835_I2C_DLEN: 0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DLEN));
	// Read the interrupt status2835_I2C_A:    0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_A));
	val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_FIFO));
	if (val == 0) {->dev, "  BCM2835_I2C_DIV:  0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DIV));
		// Spurious interrupt, increment counter  0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_DEL));
		spurious_count++;ev, "  BCM2835_I2C_CLKT: 0x%x\n", bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_CLKT));
		dev_warn(i2c_dev->dev, "Spurious IRQ, status: 0x%x (count: %d)\n", val, spurious_count);

		// Disable IRQ if spurious interrupts exceed threshold
		if (spurious_count > 10) {dev);
			dev_err(i2c_dev->dev, "Disabling IRQ due to repeated spurious interrupts\n");
			disable_irq_nosync(i2c_dev->irq);pts exceed threshold
		}f (spurious_count > 10) {atic int spurious_count = 0;
		return IRQ_NONE;->dev, "Disabling IRQ due to repeated spurious interrupts\n");
	}	disable_irq_nosync(i2c_dev->irq);/ Read the interrupt status
		}	val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
	// Reset spurious interrupt counter
	spurious_count = 0;nter
		spurious_count++;
	dev_dbg(i2c_dev->dev, "ISR triggered, status: 0x%x\n", val);nt);
	spurious_count = 0;
	// Check for errorspt
	err = val & (BCM2835_I2C_S_CLKT | BCM2835_I2C_S_ERR);, val);
	if (err) {
		dev_err(i2c_dev->dev, "ISR error: 0x%x\n", err);
		i2c_dev->msg_err = err;_S_CLKT | BCM2835_I2C_S_ERR);) {
		goto complete;ev, "Disabling IRQ due to repeated spurious interrupts\n");
	}dev_err(i2c_dev->dev, "ISR error: 0x%x\n", err);	disable_irq_nosync(i2c_dev->irq);
		i2c_dev->msg_err = err;		}
	// Check for transfer complete
	if (val & BCM2835_I2C_S_DONE) {
		dev_dbg(i2c_dev->dev, "ISR transfer complete\n");
		if (!i2c_dev->curr_msg) {letet counter
			dev_err(i2c_dev->dev, "Unexpected interrupt (no current message)\n");
		} else if (i2c_dev->curr_msg->flags & I2C_M_RD) {
			bcm2835_drain_rxfifo(i2c_dev);status: 0x%x\n", val);
			val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);current message)\n");
		} else if (i2c_dev->curr_msg->flags & I2C_M_RD) { Check for errors
			bcm2835_drain_rxfifo(i2c_dev);	err = val & (BCM2835_I2C_S_CLKT | BCM2835_I2C_S_ERR);
		if ((val & BCM2835_I2C_S_RXD) || i2c_dev->msg_buf_remaining)
			i2c_dev->msg_err = BCM2835_I2C_S_LEN;
		elsesg_err = err;
			i2c_dev->msg_err = 0;_S_RXD) || i2c_dev->msg_buf_remaining)
		goto complete;err = BCM2835_I2C_S_LEN;
	}else
			i2c_dev->msg_err = 0;	// Check for transfer complete
	// Handle TX FIFO empty
	if (val & BCM2835_I2C_S_TXW) {
		dev_dbg(i2c_dev->dev, "ISR TX FIFO empty\n");
		if (!i2c_dev->msg_buf_remaining) {no current message)\n");
			i2c_dev->msg_err = val | BCM2835_I2C_S_LEN;
			goto complete;->dev, "ISR TX FIFO empty\n");rxfifo(i2c_dev);
		}f (!i2c_dev->msg_buf_remaining) {val = bcm2835_i2c_readl(i2c_dev, BCM2835_I2C_S);
			i2c_dev->msg_err = val | BCM2835_I2C_S_LEN;		}
		bcm2835_fill_txfifo(i2c_dev);
		}		if ((val & BCM2835_I2C_S_RXD) || i2c_dev->msg_buf_remaining)
		if (i2c_dev->num_msgs && !i2c_dev->msg_buf_remaining) {
			i2c_dev->curr_msg++;2c_dev);
			bcm2835_i2c_start_transfer(i2c_dev);
		}f (i2c_dev->num_msgs && !i2c_dev->msg_buf_remaining) {oto complete;
			i2c_dev->curr_msg++;	}
		return IRQ_HANDLED;transfer(i2c_dev);
	}}/ Handle TX FIFO empty
	if (val & BCM2835_I2C_S_TXW) {
	// Handle RX FIFO fullISR TX FIFO empty\n");
	if (val & BCM2835_I2C_S_RXR) {
		dev_dbg(i2c_dev->dev, "ISR RX FIFO full\n");
		if (!i2c_dev->msg_buf_remaining) {
			i2c_dev->msg_err = val | BCM2835_I2C_S_LEN;
			goto complete;->dev, "ISR RX FIFO full\n");
		}f (!i2c_dev->msg_buf_remaining) {cm2835_fill_txfifo(i2c_dev);
			i2c_dev->msg_err = val | BCM2835_I2C_S_LEN;
		bcm2835_drain_rxfifo(i2c_dev);maining) {
		return IRQ_HANDLED;
	}cm2835_i2c_start_transfer(i2c_dev);
		bcm2835_drain_rxfifo(i2c_dev);		}
	// If no conditions are met, return IRQ_NONE
	dev_warn(i2c_dev->dev, "Unhandled IRQ, status: 0x%x\n", val);
	return IRQ_NONE;
	// If no conditions are met, return IRQ_NONE
complete:(i2c_dev->dev, "Unhandled IRQ, status: 0x%x\n", val);e RX FIFO full
	// Clear the interrupt and signal completion
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, BCM2835_I2C_C_CLEAR);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_S, BCM2835_I2C_S_CLKT |
			   BCM2835_I2C_S_ERR | BCM2835_I2C_S_DONE);
	complete(&i2c_dev->completion);2835_I2C_C, BCM2835_I2C_C_CLEAR);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_S, BCM2835_I2C_S_CLKT |		}
	return IRQ_HANDLED;ERR | BCM2835_I2C_S_DONE);
}complete(&i2c_dev->completion);	bcm2835_drain_rxfifo(i2c_dev);
		return IRQ_HANDLED;
static int bcm2835_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			    int num)
{/ If no conditions are met, return IRQ_NONE
	struct bcm2835_i2c_dev *i2c_dev = i2c_get_adapdata(adap);ct i2c_msg msgs[],al);
	unsigned long time_left;
	int i;
	struct bcm2835_i2c_dev *i2c_dev = i2c_get_adapdata(adap);complete:
	for (i = 0; i < (num - 1); i++)letion
		if (msgs[i].flags & I2C_M_RD) {LEAR);
			dev_warn_once(i2c_dev->dev,KT |
				      "only one read message supported, has to be last\n");
			return -EOPNOTSUPP;I2C_M_RD) {mpletion);
		}dev_warn_once(i2c_dev->dev,
				      "only one read message supported, has to be last\n");	return IRQ_HANDLED;
	i2c_dev->curr_msg = msgs;
	i2c_dev->num_msgs = num;
	reinit_completion(&i2c_dev->completion);
	i2c_dev->curr_msg = msgs;			    int num)
	bcm2835_i2c_start_transfer(i2c_dev);
	reinit_completion(&i2c_dev->completion);	struct bcm2835_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	time_left = wait_for_completion_timeout(&i2c_dev->completion,
						adap->timeout);ansfer(i2c_dev);

	bcm2835_i2c_finish_transfer(i2c_dev);ut(&i2c_dev->completion,
						adap->timeout);		if (msgs[i].flags & I2C_M_RD) {
	if (!time_left) {
		bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, be last\n");
				   BCM2835_I2C_C_CLEAR);
		return -ETIMEDOUT;
	}bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C,
				   BCM2835_I2C_C_CLEAR);	i2c_dev->curr_msg = msgs;
	if (!i2c_dev->msg_err)
		return num;dev->completion);

	dev_dbg(i2c_dev->dev, "i2c transfer failed: %x\n", i2c_dev->msg_err);
		return num;
	if (i2c_dev->msg_err & BCM2835_I2C_S_ERR)
		return -EREMOTEIO;v, "i2c transfer failed: %x\n", i2c_dev->msg_err);;

	return -EIO;>msg_err & BCM2835_I2C_S_ERR)finish_transfer(i2c_dev);
}	return -EREMOTEIO;
	if (!time_left) {
static u32 bcm2835_i2c_func(struct i2c_adapter *adap)
{			   BCM2835_I2C_C_CLEAR);
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}tatic u32 bcm2835_i2c_func(struct i2c_adapter *adap)}
{
static const struct i2c_algorithm bcm2835_i2c_algo = {
	.master_xfer = bcm2835_i2c_xfer, // Updated member name
	.functionality = bcm2835_i2c_func,
};atic const struct i2c_algorithm bcm2835_i2c_algo = {ev_dbg(i2c_dev->dev, "i2c transfer failed: %x\n", i2c_dev->msg_err);
	.master_xfer = bcm2835_i2c_xfer, // Updated member name
/*functionality = bcm2835_i2c_func,f (i2c_dev->msg_err & BCM2835_I2C_S_ERR)
 * The BCM2835 was reported to have problems with clock stretching:
 * https://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html
 * https://www.raspberrypi.org/forums/viewtopic.php?p=146272
 */The BCM2835 was reported to have problems with clock stretching:
static const struct i2c_adapter_quirks bcm2835_i2c_quirks = {.html
	.flags = I2C_AQ_NO_CLK_STRETCH,orums/viewtopic.php?p=146272ct i2c_adapter *adap)
};/
static const struct i2c_adapter_quirks bcm2835_i2c_quirks = {	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
static int bcm2835_i2c_probe(struct platform_device *pdev)
{;
	struct bcm2835_i2c_dev *i2c_dev;
	int ret;t bcm2835_i2c_probe(struct platform_device *pdev)xfer = bcm2835_i2c_xfer, // Updated member name
	struct i2c_adapter *adap;
	struct clk *mclk;c_dev *i2c_dev;
	u32 bus_clk_rate = I2C_MAX_STANDARD_MODE_FREQ; // Default clock frequency
	struct i2c_adapter *adap;/*
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)ate = I2C_MAX_STANDARD_MODE_FREQ; // Default clock frequency.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html
		return -ENOMEM;ums/viewtopic.php?p=146272
	platform_set_drvdata(pdev, i2c_dev);izeof(*i2c_dev), GFP_KERNEL);
	i2c_dev->dev = &pdev->dev;cm2835_i2c_quirks = {
	init_completion(&i2c_dev->completion);
	platform_set_drvdata(pdev, i2c_dev);};
	i2c_dev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c_dev->regs))completion);e(struct platform_device *pdev)
		return PTR_ERR(i2c_dev->regs);
	i2c_dev->regs = devm_platform_ioremap_resource(pdev, 0);	struct bcm2835_i2c_dev *i2c_dev;
	mclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(mclk)) {_dev->regs);*adap;
		dev_warn(&pdev->dev, "Clock not found, using default rate\n");
		mclk = NULL; // Fallback to no clock;ODE_FREQ; // Default clock frequency
	}f (IS_ERR(mclk)) {
		dev_warn(&pdev->dev, "Clock not found, using default rate\n");	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	i2c_dev->bus_clk = bcm2835_i2c_register_div(&pdev->dev, mclk, i2c_dev);
	if (IS_ERR(i2c_dev->bus_clk)) {
		ret = PTR_ERR(i2c_dev->bus_clk);
		dev_err(&pdev->dev, "Could not register clock: %d\n", ret);, i2c_dev);
		goto err_put_exclusive_rate; {pletion);
	}ret = PTR_ERR(i2c_dev->bus_clk);
		dev_err(&pdev->dev, "Could not register clock: %d\n", ret);	i2c_dev->regs = devm_platform_ioremap_resource(pdev, 0);
	// Use ACPI to retrieve clock frequency if available
	if (device_property_read_u32(&pdev->dev, "clock-frequency", &bus_clk_rate))
		dev_warn(&pdev->dev, "Using default clock frequency\n");
	// Use ACPI to retrieve clock frequency if available	mclk = devm_clk_get(&pdev->dev, NULL);
	ret = clk_set_rate_exclusive(i2c_dev->bus_clk, bus_clk_rate);bus_clk_rate))
	if (ret < 0)dev->dev, "Using default clock frequency\n");dev->dev, "Clock not found, using default rate\n");
		return dev_err_probe(&pdev->dev, ret,
				     "Could not set clock frequency\n");lk, bus_clk_rate);
	if (ret < 0)
	ret = clk_prepare_enable(i2c_dev->bus_clk);v->dev, mclk, i2c_dev);
	if (ret) {ould not set clock frequency\n");(i2c_dev->bus_clk)) {
		dev_err(&pdev->dev, "Couldn't prepare clock"););
		goto err_put_exclusive_rate;dev->bus_clk);ot register clock: %d\n", ret);
	}f (ret) {goto err_put_exclusive_rate;
		dev_err(&pdev->dev, "Couldn't prepare clock");	}
	i2c_dev->irq = platform_get_irq(pdev, 0);n", clk_get_rate(i2c_dev->bus_clk));
	if (i2c_dev->irq < 0) {
		ret = i2c_dev->irq;orm_get_irq(pdev, 0);"clock-frequency", &bus_clk_rate))
		goto err_disable_unprepare_clk;ault clock frequency\n");
	}ret = i2c_dev->irq;f (i2c_dev->irq < 0) {
		goto err_disable_unprepare_clk;		ret = i2c_dev->irq;	ret = clk_set_rate_exclusive(i2c_dev->bus_clk, bus_clk_rate);
	dev_info(&pdev->dev, "IRQ %d registered for BCM2835 I2C\n", i2c_dev->irq);
	}		return dev_err_probe(&pdev->dev, ret,
	ret = request_irq(i2c_dev->irq, bcm2835_i2c_isr, IRQF_SHARED | IRQF_NO_SUSPEND,2c_dev->irq);
			  dev_name(&pdev->dev), i2c_dev);q);
	if (ret) {est_irq(i2c_dev->irq, bcm2835_i2c_isr, IRQF_SHARED,ble(i2c_dev->bus_clk);&pdev->dev, "IRQ %d might be shared or misconfigured\n", i2c_dev->irq);
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto err_disable_unprepare_clk;
	}dev_err(&pdev->dev, "Could not request IRQ\n");f (ret) {goto err_put_exclusive_rate;et = request_irq(i2c_dev->irq, bcm2835_i2c_isr, IRQF_SHARED,
		goto err_disable_unprepare_clk;		dev_err(&pdev->dev, "Could not request IRQ\n");	}			  dev_name(&pdev->dev), i2c_dev);
	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	snprintf(adap->name, sizeof(adap->name), "bcm2835 (%s)", dev_name(&pdev->dev));
	adap->algo = &bcm2835_i2c_algo;TED;
	adap->dev.parent = &pdev->dev;ap->name), "bcm2835 (%s)", dev_name(&pdev->dev));ATED;
	adap->quirks = &bcm2835_i2c_quirks;, "bcm2835 (%s)", dev_name(&pdev->dev));ered for BCM2835 I2C\n", i2c_dev->irq);
	adap->dev.parent = &pdev->dev;	adap->algo = &bcm2835_i2c_algo;	adap->class = I2C_CLASS_DEPRECATED;
	// Disable hardware clock stretching timeout(%s)", dev_name(&pdev->dev));
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_CLKT, 0);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, 0);
	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_CLKT, 0);	// Disable hardware clock stretching timeout		dev_err(&pdev->dev, "Could not request IRQ\n");	adap->quirks = &bcm2835_i2c_quirks;
	ret = i2c_add_adapter(adap);BCM2835_I2C_C, 0);BCM2835_I2C_CLKT, 0);clk;
	if (ret)l(i2c_dev, BCM2835_I2C_C, 0);ware clock stretching timeout
		goto err_free_irq;er(adap);
	if (ret)	ret = i2c_add_adapter(adap);	adap = &i2c_dev->adapter;	bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, 0);
	return 0;_free_irq;apdata(adap, i2c_dev);
		goto err_free_irq;	adap->owner = THIS_MODULE;	ret = i2c_add_adapter(adap);
err_free_irq:ECATED;
	free_irq(i2c_dev->irq, i2c_dev);
err_disable_unprepare_clk:
	clk_disable_unprepare(i2c_dev->bus_clk);
err_put_exclusive_rate:lk: i2c_dev);5_i2c_quirks;
	clk_rate_exclusive_put(i2c_dev->bus_clk);
err_put_exclusive_rate:	clk_disable_unprepare(i2c_dev->bus_clk);	// Disable hardware clock stretching timeout	free_irq(i2c_dev->irq, i2c_dev);
	return ret;clusive_put(i2c_dev->bus_clk);usive_rate:_writel(i2c_dev, BCM2835_I2C_CLKT, 0);unprepare_clk:
}lk_rate_exclusive_put(i2c_dev->bus_clk);bcm2835_i2c_writel(i2c_dev, BCM2835_I2C_C, 0);clk_disable_unprepare(i2c_dev->bus_clk);
	return ret;err_put_exclusive_rate:
static int bcm2835_i2c_remove(struct platform_device *pdev) // Changed return type to int
{f (ret)
	struct bcm2835_i2c_dev *i2c_dev = platform_get_drvdata(pdev); Changed return type to int
{static int bcm2835_i2c_remove(struct platform_device *pdev) // Changed return type to int}
	clk_rate_exclusive_put(i2c_dev->bus_clk);m_get_drvdata(pdev);
	clk_disable_unprepare(i2c_dev->bus_clk);
	clk_rate_exclusive_put(i2c_dev->bus_clk);err_free_irq:{
	free_irq(i2c_dev->irq, i2c_dev);us_clk);bus_clk);= platform_get_drvdata(pdev);
	i2c_del_adapter(&i2c_dev->adapter);
	free_irq(i2c_dev->irq, i2c_dev);	clk_disable_unprepare(i2c_dev->bus_clk);	clk_rate_exclusive_put(i2c_dev->bus_clk);
	return 0; // Added return statement
}2c_del_adapter(&i2c_dev->adapter);clk_rate_exclusive_put(i2c_dev->bus_clk);
	return 0; // Added return statement	free_irq(i2c_dev->irq, i2c_dev);
static const struct acpi_device_id bcm2835_i2c_acpi_match[] = {
	{ "BCM2835", 0 },
	{ "BCM2711", 0 },t acpi_device_id bcm2835_i2c_acpi_match[] = {
	{},BCM2835", 0 },ic const struct acpi_device_id bcm2835_i2c_acpi_match[] = {ic int bcm2835_i2c_remove(struct platform_device *pdev) // Changed return type to int
}; "BCM2711", 0 }, "BCM2835", 0 },
MODULE_DEVICE_TABLE(acpi, bcm2835_i2c_acpi_match);
};	{},	{ "BCM2835", 0 },
static struct platform_driver bcm2835_i2c_driver = {
	.probe		= bcm2835_i2c_probe,lk);
	.remove		= bcm2835_i2c_remove,cm2835_i2c_driver = {
	.driver		= {m2835_i2c_probe, platform_driver bcm2835_i2c_driver = {_dev->irq, i2c_dev);_TABLE(acpi, bcm2835_i2c_acpi_match);
		.name	= "i2c-bcm2835",remove,robe,v->adapter);
		.acpi_match_table = ACPI_PTR(bcm2835_i2c_acpi_match),
	},name	= "i2c-bcm2835",river		= {turn 0; // Added return statementrobe		= bcm2835_i2c_probe,
};.acpi_match_table = ACPI_PTR(bcm2835_i2c_acpi_match),.name	= "i2c-bcm2835",emove		= bcm2835_i2c_remove,
module_platform_driver(bcm2835_i2c_driver);
};	},static const struct acpi_device_id bcm2835_i2c_acpi_match[] = {		.name	= "i2c-bcm2835",
MODULE_AUTHOR("Stephen Warren <swarren@wwwdotorg.org>");
MODULE_DESCRIPTION("BCM2835 I2C bus adapter");
MODULE_LICENSE("GPL v2");rren <swarren@wwwdotorg.org>");
MODULE_ALIAS("platform:i2c-bcm2835");dapter");n@wwwdotorg.org>");
MODULE_LICENSE("GPL v2");MODULE_DESCRIPTION("BCM2835 I2C bus adapter");MODULE_DEVICE_TABLE(acpi, bcm2835_i2c_acpi_match);


MODULE_ALIAS("platform:i2c-bcm2835");


MODULE_ALIAS("platform:i2c-bcm2835");MODULE_LICENSE("GPL v2");















MODULE_ALIAS("platform:i2c-bcm2835");MODULE_LICENSE("GPL v2");MODULE_DESCRIPTION("BCM2835 I2C bus adapter");MODULE_AUTHOR("Stephen Warren <swarren@wwwdotorg.org>");module_platform_driver(bcm2835_i2c_driver);};	},		.acpi_match_table = ACPI_PTR(bcm2835_i2c_acpi_match),		.name	= "i2c-bcm2835",	.driver		= {	.remove		= bcm2835_i2c_remove,	.probe		= bcm2835_i2c_probe,static struct platform_driver bcm2835_i2c_driver = {MODULE_AUTHOR("Stephen Warren <swarren@wwwdotorg.org>");
MODULE_DESCRIPTION("BCM2835 I2C bus adapter");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:i2c-bcm2835");
