/*
 * sunxi-lirc-pwm.c
 *
 * sunxi-lirc-pwm - Device driver for LIRC using Allwinner A1X or A20
 * 					IR module in CIR mode for receiving and PWM outputs
 * 					for transmitting IR signal.
 * 					A lot of code for receiving part was reused from
 * 					sunxi-lirc driver (https://github.com/matzrh/sunxi-lirc).
 * 					Driver was tested on a Cubietruck with Allwinner A20.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/preempt.h>
#include <plat/irqs.h>
#include <asm-generic/errno.h>
#include <asm/uaccess.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>

#define DRIVER_NAME "sunxi_lirc_pwm"
#define LIRC_READBUF_LEN 	256
#define	IR_RAW_BUF_SIZE		128

// Registers & fields
#define PORT_IO_BASE		0xf1c20800
#define PI_CFG0				PORT_IO_BASE + 0x120
#define PI_DAT				PORT_IO_BASE + 0x130
#define PI3_MUX_SHIFT		12

#define PB_CFG0				PORT_IO_BASE + 0x24
#define PB_DAT				PORT_IO_BASE + 0x34
#define PB2_MUX_SHIFT		8

#define PWM_PORT_CFG(x)  	(x ? PI_CFG0 : PB_CFG0)
#define PWM_PORT_DAT(x)  	(x ? PI_DAT : PB_DAT)
#define PWM_MUX_SHIFT(x) 	(x ? PI3_MUX_SHIFT : PB2_MUX_SHIFT)

#define PWM_BASE			0xf1c20c00
#define PWM_CTRL_REG		PWM_BASE + 0x200
#define PWM_PERIOD(x)		(PWM_BASE + 0x204 + 4 * x)

#define PWM_BYPASS(x)		(9 + 15 * x)
#define PWM_CLK_GATING(x)	(6 + 15 * x)
#define PWM_ACT_STATE(x)	(5 + 15 * x)
#define PWM_EN(x)			(4 + 15 * x)
#define PWM_PRESCAL(x)		(15 * x)

#define PWM_CLOCK			24000000

#define CCU_BASE			0xf1c20000
#define IR0_CLK_REG			CCU_BASE + 0xB0
#define ABP0_GATING_REG		CCU_BASE + 0x68

#define MAX_UDELAY_US		1000

/* IR registers */
#define IR0_BASE			(0xf1c21800)
#define IR_BASE				IR0_BASE
#define IR_CTL				IR_BASE + 0x00
#define IR_RXCTL			IR_BASE + 0x10
#define IR_RXFIFO			IR_BASE + 0x20
#define IR_RXINT			IR_BASE + 0x2C
#define IR_RXSTA			IR_BASE + 0x30
#define IR_CIR  			IR_BASE + 0x34
#define IR_IRQNO			(SW_INT_IRQNO_IR0)

/* Helper */
#define PULSE_BIT_SHIFTER	(17)  /* from 0x80 to PULSE_BIT */
#define SAMPLES_TO_US(us)	(( ((unsigned long) us) * 1000000UL) / 46875UL)
#define RXPACKET_FIRST_SPACE_SAMPLE 	1

#define dprintk(fmt, args...)                             \
do {                                                      \
	if (debug)                                            \
		printk(KERN_DEBUG DRIVER_NAME ": " fmt, ## args); \
} while (0)

struct pwm_params {
		unsigned short channel;
		unsigned long freq;
		unsigned int duty_cycle;
};

struct ir_raw_buffer {
	unsigned int dcnt;	/*Packet Count*/
	unsigned char buf[IR_RAW_BUF_SIZE];
};

static struct pwm_params *pwm;
static unsigned short pwm_channel = 0;
static struct platform_device *lirc_sunxi_dev;
static struct lirc_buffer readbuf;
static struct lirc_driver driver;
static struct ir_raw_buffer ir_rawbuf;
static struct timer_list rx_timer;
static unsigned short rx_state = 0;

#ifdef SYS_CLK_CFG_EN
		static struct clk *apb_ir_clk;
		static struct clk *ir_clk;
#endif

#ifdef SYS_GPIO_CFG_EN
		static u32 ir_gpio_hdle;
#endif

static int debug=0;

DEFINE_SPINLOCK(sunxi_lirc_spinlock);

/*
 *	TX PWM stuff
 */
static void configure_tx_pin(void)
{
		unsigned long tmp = 0;

		tmp = readl(PWM_PORT_CFG(pwm->channel));
		tmp |= (2 << PWM_MUX_SHIFT(pwm->channel));
		writel(tmp, PWM_PORT_CFG(pwm->channel));
}

static void unconfigure_tx_pin(void)
{
		unsigned long tmp = 0;

		tmp = readl(PWM_PORT_CFG(pwm->channel));
		tmp &= ~(7 << PWM_MUX_SHIFT(pwm->channel));
		writel(tmp, PWM_PORT_CFG(pwm->channel));
}

static void pwm_enable(void)
{
		unsigned long tmp = 0;
		tmp = readl(PWM_CTRL_REG);
		tmp |= (1 << PWM_EN(pwm->channel));
		writel(tmp, PWM_CTRL_REG);
}

static void pwm_disable(void)
{
		unsigned long tmp = 0;
		tmp = readl(PWM_CTRL_REG);
		tmp &= ~(1 << PWM_EN(pwm->channel));
		writel(tmp, PWM_CTRL_REG);
}

static void setup_pwm_reg(void)
{
		unsigned long tmp = 0;

		tmp = readl(PWM_CTRL_REG);
		tmp |= (1 << PWM_CLK_GATING(pwm->channel))
			 | (1 << PWM_ACT_STATE(pwm->channel))
			 | (0xF << PWM_PRESCAL(pwm->channel));
		writel(tmp, PWM_CTRL_REG);
		dprintk("PWM config register: 0x%lx \n", tmp);
}

static void clear_pwm_reg(void)
{
		unsigned long tmp = 0;

		tmp = readl(PWM_CTRL_REG);
		tmp &= ~(1 << PWM_CLK_GATING(pwm->channel));
		tmp &= ~(1 << PWM_ACT_STATE(pwm->channel));
		tmp &= ~(1 << PWM_EN(pwm->channel));
		tmp &= ~(0xF << PWM_PRESCAL(pwm->channel));
		writel(tmp, PWM_CTRL_REG);
		dprintk("PWM config register: 0x%lx \n", tmp);
}

static void set_pwm_timing(struct pwm_params *pwm_tparam)
{
		unsigned long tmp = 0, pCnt = 0, pActiveCnt = 0;

		pwm_disable();

		printk(KERN_INFO DRIVER_NAME ": Setting PWM timing parameters: Carrier: %luHz, Duty cycle: %i%% \n",
							pwm_tparam->freq,
							pwm_tparam->duty_cycle);

		pCnt = PWM_CLOCK / (pwm_tparam->freq);
		pActiveCnt = pCnt * pwm_tparam->duty_cycle / 100;

		dprintk("PWM clock entire cycles in period: %lu \n", pCnt);
		dprintk("PWM clock active cycles in period: %lu \n", pActiveCnt);

		tmp = (pCnt << 16) | pActiveCnt;
		writel(tmp, PWM_PERIOD(pwm->channel));
}

static void setup_ir_tx(void)
{
		pwm = kzalloc(sizeof(struct pwm_params), GFP_KERNEL);
		pwm->channel = pwm_channel;
		pwm->freq = 38000;
		pwm->duty_cycle = 50;
		configure_tx_pin();
		set_pwm_timing(pwm);
		setup_pwm_reg();
}

static void free_ir_tx(void)
{
		unconfigure_tx_pin();
		clear_pwm_reg();
		kfree(pwm);
}

static inline void safe_udelay(unsigned long usecs)
{
        while (usecs > MAX_UDELAY_US) {
                udelay(MAX_UDELAY_US);
                usecs -= MAX_UDELAY_US;
        }
        udelay(usecs);
}

static ssize_t lirc_write(struct file *file, const char *buf, size_t n, loff_t *ppos)
{
		int i, count;
		int *txbuf;

		count = n / sizeof(int);
		if (n % sizeof(int) || count % 2 == 0)
				return -EINVAL;

		txbuf = memdup_user(buf, n);
		if (IS_ERR(txbuf))
				return PTR_ERR(txbuf);

        dprintk("Sending (%i) pulses \n", count);

        preempt_disable();
		for (i = 0; i < count; i++) {
				if (i % 2)
						pwm_disable();
				else
						pwm_enable();
				safe_udelay(txbuf[i]);
		}
		pwm_disable();
		preempt_enable();

		dprintk("Sending (%i) pulses completed \n", count);

		kfree(txbuf);
		return n;
}
/*
 *	End of TX PWM stuff
 */

/*
 * RX stuff
 */

static inline void ir_reset_rawbuffer(void)
{
		ir_rawbuf.dcnt = 0;
}

static inline void ir_write_rawbuffer(unsigned char data)
{
		if (ir_rawbuf.dcnt < IR_RAW_BUF_SIZE)
				ir_rawbuf.buf[ir_rawbuf.dcnt++] = data;
		else
				printk(KERN_INFO DRIVER_NAME ": ir_write_rawbuffer: IR RX buffer full \n");
}

static inline int ir_rawbuffer_full(void)
{
		return (ir_rawbuf.dcnt >= IR_RAW_BUF_SIZE);
}

static void ir_clk_cfg(void)
{
#ifdef SYS_CLK_CFG_EN
		unsigned long rate = 3000000; /* 3 MHz */
		apb_ir_clk = clk_get(NULL, "apb_ir0");
		if (!apb_ir_clk) {
				printk(KERN_ERR DRIVER_NAME ": try to get apb_ir0 clock failed\n");
				return;
		}

		ir_clk = clk_get(NULL, "ir0");
		if (!ir_clk) {
				printk(KERN_ERR DRIVER_NAME ": try to get ir0 clock failed\n");
				return;
		}
		printk(KERN_INFO DRIVER_NAME ": trying to set clock via SYS_CLK_CFG_EN\n");
		if (clk_set_rate(ir_clk, rate))
				printk(KERN_ERR DRIVER_NAME ": set ir0 clock freq to 3M failed\n");

		if (clk_enable(apb_ir_clk))
				printk(KERN_ERR DRIVER_NAME ": try to enable apb_ir_clk failed\n");

		if (clk_enable(ir_clk))
				printk(KERN_ERR DRIVER_NAME ": try to enable apb_ir_clk failed\n");

#else
		unsigned long tmp = 0;
		printk(KERN_INFO DRIVER_NAME ":setting clock via register manipulation\n");
		/* Enable APB Clock for IR */

		tmp = readl(ABP0_GATING_REG);
		tmp |= (1 << 6);
		writel(tmp, ABP0_GATING_REG);

		tmp = readl(IR0_CLK_REG);
		tmp |= (1 << 31) | (3 << 16); 	//SCLK_GATING | CLK_DIV_RATIO_N = 8
		tmp &= ~(0x03 << 24); 			// OSC24M
		tmp &= ~(0x0F << 0);  			//CLK_DIV_RATIO_M = 1
		writel(tmp, IR0_CLK_REG);

#endif
}

static void ir_clk_uncfg(void)
{
#ifdef SYS_CLK_CFG_EN
		clk_put(apb_ir_clk);
		clk_put(ir_clk);
#endif
}

static void ir_sys_cfg(void)
{
#ifdef SYS_GPIO_CFG_EN
		ir_gpio_hdle = gpio_request_ex("ir_para", "ir0_rx");
		if (0 == ir_gpio_hdle)
				printk(KERN_ERR DRIVER_NAME ": try to request ir_para gpio failed \n");
#else
		/* config IO: PB4 to IR_RX */
		unsigned long tmp = 0;
		tmp = readl(PB_CFG0);
		tmp &= ~(0xf << 16);
		tmp |= (0x2 << 16);
		writel(tmp, PB_CFG0);
#endif
		ir_clk_cfg();
}

static void ir_sys_uncfg(void)
{
#ifdef SYS_GPIO_CFG_EN
		gpio_release(ir_gpio_hdle, 2);
#endif
		ir_clk_uncfg();
}

static void ir_reg_cfg(void)
{
		unsigned long tmp = 0;
		/* Enable CIR Mode */
		tmp = 0x3 << 4;
		writel(tmp, IR_CTL);

		/* Config IR Sample Register */
		tmp = 0x0 << 0; /* Fsample = 3MHz/64 =46875Hz (21.3us) */

		tmp |= (8 & 0x3f) << 2; /* Set Filter Threshold */
		tmp |= (5 & 0xff) << 8; /* Set Idle Threshold */
		writel(tmp, IR_CIR);

		/* Invert Input Signal */
		writel(0x1 << 2, IR_RXCTL);

		/* Clear All Rx Interrupt Status */
		writel(0xff, IR_RXSTA);

		/* Set Rx Interrupt Enable */
		tmp = (0x1 << 4) | 0x3;
#ifdef CONFIG_ARCH_SUN5I
		tmp |= ((16 >> 2) - 1) << 8; /* Rx FIFO Threshold = FIFOsz/4 */
#else
		tmp |= ((16 >> 1) - 1) << 8; /* Rx FIFO Threshold = FIFOsz/2 */
#endif
		writel(tmp, IR_RXINT);

		/* Enable IR Module */
		tmp = readl(IR_CTL);
		tmp |= 0x3;
		writel(tmp, IR_CTL);
}

static void ir_setup(void)
{
		dprintk("ir_setup: ir setup start \n");

		ir_reset_rawbuffer();
		ir_sys_cfg();
		ir_reg_cfg();

		dprintk("ir_setup: ir setup comleted \n");
}

static inline unsigned char ir_get_data(void)
{
		return (unsigned char)(readl(IR_RXFIFO));
}

static inline unsigned long ir_get_intsta(void)
{
		return readl(IR_RXSTA);
}

static inline void ir_clr_intsta(unsigned long bitmap)
{
		unsigned long tmp = readl(IR_RXSTA);

		tmp &= ~0xff;
		tmp |= bitmap&0xff;
		writel(tmp, IR_RXSTA);
}

void ir_packet_handler(unsigned char *buf, unsigned int dcnt)
{
		unsigned int i = 0;
		unsigned  int lirc_val = 0;
		dprintk("Buffer length: %d", dcnt);

		for(; i < dcnt; i++) {
				lirc_val = ((unsigned int) (buf[i] & 0x80) << PULSE_BIT_SHIFTER) | (SAMPLES_TO_US(buf[i] &0x7f));
				while((buf[i] & 0x80) == (buf[i+1] & 0x80)) {
					lirc_val += SAMPLES_TO_US(buf[++i]&0x7f);
				}

				/* statistically pulses are one sample period (?) too long, spaces too short */
				/* would make sense because of bandpass latency, but not sure... */
				lirc_val += (buf[i] & 0x80) ? (-SAMPLES_TO_US(1)) : SAMPLES_TO_US(1);
				dprintk("rawbuf: %x (%i), value: %x, level:%d for %d us\n",buf[i],buf[i],lirc_val,(buf[i]&0x80) ? 1 : 0,lirc_val & PULSE_MASK) ;
				//dprintk("%s %d \n", (buf[i]&0x80) ? "pulse" : "space", lirc_val & PULSE_MASK);

				if (lirc_buffer_full(&readbuf)) {
						/* no new signals will be accepted */
						dprintk("Buffer overrun\n");
						return;
				}
				lirc_buffer_write(&readbuf,(unsigned char*)&lirc_val);
		}
}

void rx_timer_expired( unsigned long data )
{
	dprintk("Receiving packet stopped \n");
	rx_state = 0;
}

static inline void restart_receiving_timer(void) {
	dprintk("Restarting receiving timer \n");
	mod_timer(&rx_timer, jiffies + msecs_to_jiffies(200)); // wait 200ms after last lirc buffer write
}

static irqreturn_t ir_irq_service(int irqno, void *dev_id)
{
		unsigned int dcnt, i;
		unsigned long intsta;

		intsta = ir_get_intsta();
		ir_clr_intsta(intsta);

		/* Read Data Every Time Enter this Routine*/
#ifdef CONFIG_ARCH_SUN5I
		dcnt = (unsigned int) (ir_get_intsta() >> 8) & 0x3f;
#else
		dcnt = (unsigned int) (ir_get_intsta() >> 8) & 0x1f;
#endif

		// LIRC require space as a first value in buffer
		if(rx_state == 0) {
			rx_state = 1;
			if(ir_rawbuf.dcnt == 0) {
				dprintk("Writing first space to raw buffer \n");
				ir_write_rawbuffer(RXPACKET_FIRST_SPACE_SAMPLE);
			}
		}

		/* Read FIFO */
		for (i = 0; i < dcnt; i++) {
				if (ir_rawbuffer_full()) {
						dprintk("ir_irq_service: raw buffer full\n");
						break;
				} else {
						ir_write_rawbuffer(ir_get_data());
				}
		}

		if (intsta & (0x1 << 1)) { /* Packet End */
				ir_packet_handler(ir_rawbuf.buf, ir_rawbuf.dcnt);
				dprintk("Buffer written\n");
				restart_receiving_timer();
				ir_reset_rawbuffer();
				wake_up_interruptible(&readbuf.wait_poll);
		}

		if (intsta & 0x1) {/* FIFO Overflow */
				/* flush raw buffer */
				ir_reset_rawbuffer();
				dprintk("ir_irq_service: Rx FIFO Overflow!!\n");
		}

		return IRQ_HANDLED;
}

/*
 * End of RX stuff
 */

static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
        int result;
        __u32 value;

        switch (cmd) {
           case LIRC_SET_SEND_DUTY_CYCLE:
                result = get_user(value, (__u32 *) arg);

                if (result)
                    	return result;

                if (value <= 0 || value > 100)
                    	return -EINVAL;

                if(pwm->duty_cycle != value) {
                		dprintk("Setting send duty cycle to: %u%% \n", value);
                		pwm->duty_cycle = value;
                		set_pwm_timing(pwm);
                }

                break;

            case LIRC_SET_SEND_CARRIER:
                result = get_user(value, (__u32 *) arg);

                if (result)
                    	return result;
                if (value > 500000 || value < 20000)
                    	return -EINVAL;

                if(pwm->freq != value) {
                		dprintk("Setting send carrier to: %uHz \n", value);
                		pwm->freq = value;
                		set_pwm_timing(pwm);
                }

                break;

            default:
                return lirc_dev_fop_ioctl(filep, cmd, arg);
        }

        return 0;
}

static int set_use_inc(void* data)
{
		return 0;
}

static void set_use_dec(void* data)
{

}

static const struct file_operations lirc_fops = {
        .owner               = THIS_MODULE,
        .unlocked_ioctl      = lirc_ioctl,
        .write               = lirc_write,
        .read                = lirc_dev_fop_read, // this and the rest is default
        .poll                = lirc_dev_fop_poll,
        .open                = lirc_dev_fop_open,
        .release             = lirc_dev_fop_close,
        .llseek              = no_llseek,
};

static struct lirc_driver driver = {
        .name         = DRIVER_NAME,
        .minor        = -1,         // assing automatically
        .code_length  = 1,
        .sample_rate  = 0,
        .data         = NULL,
        .add_to_buf   = NULL,
        .rbuf         = &readbuf,
        .set_use_inc  = set_use_inc,
        .set_use_dec  = set_use_dec,
        .fops         = &lirc_fops,
        .dev          = NULL,
        .owner        = THIS_MODULE,
};

static struct platform_driver lirc_sunxi_driver = {
        .driver = {
            .name   = DRIVER_NAME,
            .owner  = THIS_MODULE,
        },
};

static int __init sunxi_lirc_pwm_init(void) {

		int result;

		printk(KERN_INFO DRIVER_NAME ": Initializing Driver\n");

		result = lirc_buffer_init(&readbuf, sizeof(int), LIRC_READBUF_LEN);
		if (result < 0)
				return -ENOMEM;

		result = platform_driver_register(&lirc_sunxi_driver);
		if (result) {
				printk(KERN_ERR DRIVER_NAME ": lirc register returned %d\n", result);
				goto exit_buffer_free;
		}

		lirc_sunxi_dev = platform_device_alloc(DRIVER_NAME, 0);
		if (!lirc_sunxi_dev) {
				result = -ENOMEM;
				goto exit_driver_unregister;
		}

		result = platform_device_add(lirc_sunxi_dev);
		if (result) {
				platform_device_put(lirc_sunxi_dev);
				goto exit_driver_unregister;
		}

		// 'driver' is the lirc driver
		driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
						  LIRC_CAN_SET_SEND_CARRIER |
						  LIRC_CAN_REC_MODE2 |
						  LIRC_CAN_SEND_PULSE;

		driver.dev = &lirc_sunxi_dev->dev;  // link THIS platform device to lirc driver
		driver.minor = lirc_register_driver(&driver);

		if (driver.minor < 0) {
				printk(KERN_ERR DRIVER_NAME ": device registration failed with %d\n", result);
				result = -EIO;
		}

		setup_timer(&rx_timer, rx_timer_expired, 0 );

		if (request_irq(IR_IRQNO, ir_irq_service, 0, "RemoteIR", (void*) 0)) {
				result = -EBUSY;
				goto exit_device_unregister;
		}

		ir_setup();
		setup_ir_tx();
		printk(KERN_INFO DRIVER_NAME ": Driver Initialized\n");

		return 0;

exit_device_unregister:
		platform_device_unregister(lirc_sunxi_dev);

exit_driver_unregister:
    	platform_driver_unregister(&lirc_sunxi_driver);

exit_buffer_free:
		lirc_buffer_free(&readbuf);

		return result;
}

static void __exit sunxi_lirc_pwm_exit(void) {
		printk(KERN_INFO DRIVER_NAME ": Cleaning Driver\n");

		del_timer(&rx_timer);
		free_irq(IR_IRQNO, (void*) 0);
		ir_sys_uncfg();

		platform_device_unregister(lirc_sunxi_dev);
		platform_driver_unregister(&lirc_sunxi_driver);

		lirc_buffer_free(&readbuf);
		lirc_unregister_driver(driver.minor);

		free_ir_tx();

		printk(KERN_INFO DRIVER_NAME ": Driver module cleaned up\n");
}

module_init(sunxi_lirc_pwm_init);
module_exit(sunxi_lirc_pwm_exit);

module_param(pwm_channel, short, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pwm_channel, "PWM Channel(0, 1) for IR TX. Default: 0");
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Andrzejewski <marek1and@gmail.com>");
MODULE_DESCRIPTION("A1X/A20 LIRC module with hardware CIR receiver and PWM transmitter");
MODULE_VERSION("1.0");
