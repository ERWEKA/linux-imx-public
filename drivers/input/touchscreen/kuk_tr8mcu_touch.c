/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a driver for the Trizeps VIII Kinetis MCU resistive touch firmware
 */

#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#define DEBUG_REPORTS                      0
#define MAX_SUPPORT_POINTS		   1

#define MCU_REG_ID               0x01
#define MCU_REG_CONTROL          0x02
#define MCU_REG_CONFIG1          0x03
#define MCU_REG_PIN_CONFIG       0x04
#define MCU_REG_PIN_SET          0x05
#define MCU_REG_PIN_GET          0x06
#define MCU_REG_ADC_CONFIG       0x10
#define MCU_REG_ADC              0x11
#define MCU_REG_LSB              0x12
#define MCU_REG_MSB              0x13

#define TOUCH_CONFIG		 0x20
#define TOUCH_WAITTIME		 0x21
#define TOUCH_XSCALE_LSB	 0x22
#define TOUCH_XSCALE_MSB	 0x23
#define TOUCH_YSCALE_LSB	 0x24
#define TOUCH_YSCALE_MSB	 0x25
#define TOUCH_XMIN_LSB		 0x26
#define TOUCH_XMIN_MSB		 0x27
#define TOUCH_YMIN_LSB		 0x28
#define TOUCH_YMIN_MSB		 0x29
#define TOUCH_XMAX_Z_LSB	 0x2A
#define TOUCH_XMAX_Z_MSB	 0x2B
#define TOUCH_YMAX_LSB		 0x2C
#define TOUCH_YMAX_MSB		 0x2D
#define TOUCH_AVERAGE		 0x2E

#define MCU_REG_CONFIG1_RESET2USBBOOT	0x01
#define MCU_REG_CONFIG1_SDVSEL			0x02

enum t_ {
	T_XSCALE,
	T_YSCALE,
	T_XMIN,
	T_XMAX,
	T_YMIN,
	T_YMAX,
	T_X,
	T_Y,
	T_XRAW,
	T_YRAW,
	T_ZRAW,
	T_THRESHOLD,
	T_SETTLE,
	T_AVERAGE,
	T_SWAPXY,
	T_ADC0,
	T_ADC1,
	T_ADC2,
	T_ADC3,
	T_TSMX,
	T_TSPX,
	T_TSMY,
	T_TSPY
};


#define KUK_SWAPXY	0x1
#define KUK_XNEG	0x2
#define KUK_YNEG	0x4
#define KUK_NAME_LEN	23

/*                    0  1  2  3   4   5   6   7   8   9  10  11, 12, 13, 14, 15 */
int gpio_to_pin[] = { 2, 4, 6, 8, 14, 16, 18, 20, 87, 97, 99, -1, -1, -1, -1, -1 };
  

int tr8_read_aux_adc(struct i2c_client *client, int adcsel);

struct mcu_gpio {
	struct gpio_chip	chip;
	struct irq_chip		irqchip;
	struct i2c_client	*client;
	struct mutex		lock;		/* protect 'out' */
	unsigned		out;		/* software latch */
	unsigned		status;		/* current status */
	unsigned int		irq_parent;
	unsigned		irq_enabled;	/* enabled irqs */
};

struct kuk_tr8mcu_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;

	int reset_pin;
    int enable_touch;
	int enable_wakeup;
	int irq_pin;
        int swapxy;

	struct mutex mutex;
	
	u16 xscale;
	u16 yscale;
	u16 xmin;
	u16 ymin;
	u16 xmax;
	u16 ymax;
	u8 threshold;
	u8 settle;
	u8 average;
        u8 disable;
	u8 reg_config1;

	char name[KUK_NAME_LEN];

        int driver_suspended;
        int driver_sendbuttonup;
        struct mcu_gpio  tr8mcugpio;

	struct regulator_desc mcu_regulator_desc;
	struct regulator_dev *mcu_regulator_dev;
};

struct kuk_tr8mcu_attribute {
	struct device_attribute dattr;
	u16 limit_low;
	u16 limit_high;
	u8 addr;
};
static int kuk_tr8mcu_ts_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;
	if( client == NULL )
	{
	    printk(KERN_ERR "kuk_tr8mcu_ts_readwrite Client=0 \n");
	    return(-1);
	}
	
	if( client->adapter == 0)
	{
	    printk(KERN_ERR "kuk_tr8mcu_ts_readwrite client->adapter=0 \n");
	    return(-1);	    	      
	}
	
	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;

	return 0;
}

static int kuk_tr8mcu_ts_read8(struct i2c_client *client, u8 reg)
{
	u8 data;
	int datalen = sizeof( data);
	int error;
	error = kuk_tr8mcu_ts_readwrite( client,
					sizeof(reg), &reg,
					datalen, (u8*)&data);
	if (error) {
		dev_err_ratelimited(&client->dev, "kuk_tr8mcu_ts_read8: Unable to fetch data, error: %d\n",
				    error);
		return 0;
	}
	return data;
}
static int kuk_tr8mcu_ts_read16(struct i2c_client *client, u8 reg)
{
	u16 data;
	int datalen = sizeof( data);
	int error;
	error = kuk_tr8mcu_ts_readwrite( client,
					sizeof(reg), &reg,
					datalen, (u8*)&data);
	if (error) {
		dev_err_ratelimited(&client->dev, "kuk_tr8mcu_ts_read16: Unable to fetch data, error: %d\n",
				    error);
		return 0;
	}
	return data;
}
static int kuk_tr8mcu_ts_read24(struct i2c_client *client, u8 reg)
{
	u32 data  = 0;
	int datalen = 3;
	int error;
	error = kuk_tr8mcu_ts_readwrite( client,
					sizeof(reg), &reg,
					datalen, (u8*)&data);
	if (error) {
		dev_err_ratelimited(&client->dev, "kuk_tr8mcu_ts_read24: Unable to fetch data, error: %d\n",
				    error);
		return 0;
	}
	return data;
}
static int kuk_tr8mcu_ts_write16(struct i2c_client *client, u8 reg, u16 data )
{
	u8 buf[3];
	buf[0] = reg;
	buf[1] = data & 0xFF;
	buf[2] = (data >> 8)&0xFF;
	return kuk_tr8mcu_ts_readwrite( client,
					3, &buf[0],
					0, NULL);
}

static int kuk_tr8mcu_ts_write8(struct i2c_client *client, u8 reg, u8 data )
{
	u8 buf[2];
	buf[0] = reg;
	buf[1] = data;
	return kuk_tr8mcu_ts_readwrite( client,
					2, &buf[0],
					0, NULL);
}


static int kuk_tr8mcu_gpio_read(struct gpio_chip *chip, unsigned int offset)
{
	struct mcu_gpio *gpio = gpiochip_get_data(chip);  
        u8  level, wbuf[4];
	int sodimmpin=gpio_to_pin[offset&0x0f];
	int error=0;
       
	if( sodimmpin == -1 )
	  return(sodimmpin);

	wbuf[0] = MCU_REG_PIN_GET;
	wbuf[1] = sodimmpin;

	level   = 0;
	error = kuk_tr8mcu_ts_readwrite(gpio->client, 2, &wbuf[0], 1, (u8*) &wbuf[0]);
	
	level = wbuf[0];
	
	printk(KERN_ERR "kuk_tr8mcu_gpio_read %d(sodimm %d)=%d [0x%02x, 0x%02x 0x%02x, 0x%02x]***\n",
               offset, sodimmpin, level,wbuf[0],wbuf[1],wbuf[2],wbuf[3]);

	if (error) {
	  dev_err(&gpio->client->dev, "Unable to fetch MCU-pin: %d err:%d\n",sodimmpin, error);
		return 0;
	}
	return level;
}

static int kuk_tr8mcu_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int level)
{
	struct mcu_gpio *gpio= gpiochip_get_data(chip);  
	u8 buf[3];
	int pin=gpio_to_pin[offset&0x0f];

	if( pin == -1 )
	  return(pin);
	
	buf[0] = MCU_REG_PIN_CONFIG;
	buf[1] = pin & 0xFF;
	if( level )   buf[2] = 21; /* Alt1-Output High */
	else          buf[2] = 20; /* Alt1-Output Low  */

	printk(KERN_ERR "kuk_tr8mcu_gpio_direction_output %d(sodimm %d)=%d ***\n", offset, pin, level);
        return(kuk_tr8mcu_ts_readwrite(gpio->client,3, &buf[0],0,NULL));	
}

static void kuk_tr8mcu_gpio_write(struct gpio_chip *chip, unsigned offset, int level)
{
#if 1
        /* Works, seems, taht mcu-FW needs a fix.... */
        kuk_tr8mcu_gpio_direction_output(chip, offset, level);
#else  
	struct mcu_gpio *gpio= gpiochip_get_data(chip);  
	u8 buf[3];
	int sodimmpin=gpio_to_pin[offset&0x0f];

	if( sodimmpin == -1 )
	  return; 
	
	buf[0] = MCU_REG_PIN_SET;
	buf[1] = sodimmpin & 0xFF;
	buf[2] = level & 0xff;
	
	printk(KERN_ERR "kuk_tr8mcu_gpio_write %d(sodimm %d) %d ***\n", offset, sodimmpin, level);	
	kuk_tr8mcu_ts_readwrite(gpio->client, 3, &buf[0], 0, NULL);
#endif	
}


static int kuk_tr8mcu_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct mcu_gpio *gpio= gpiochip_get_data(chip);
	u8 buf[3];
	int pin=gpio_to_pin[offset&0x0f];

	if( pin == -1 )
	  return(pin);

	buf[0] = MCU_REG_PIN_CONFIG;
	buf[1] = pin & 0xFF;
	buf[2] = 1; /* Alt1=Input  */

	printk(KERN_ERR "kuk_tr8mcu_gpio_direction_input %d(sodimm %d) ***\n", offset, pin);		
        return(kuk_tr8mcu_ts_readwrite(gpio->client,	3, &buf[0], 0, NULL));
}

/*
 * Touch Protocol:
 * on command, the controller sends a data load with a predefined length
 * with a predefined structure
 */
static irqreturn_t kuk_tr8mcu_ts_isr(int irq, void *dev_id)
{
	struct kuk_tr8mcu_ts_data *tsdata = dev_id;
	struct device *dev = &tsdata->client->dev;
	int x, y, datalen;
	int error;
	u8 cmd;
	u8 rdbuf[8];
 
	// printk(KERN_ERR "*** TouchScreen IRQ ***\n");

	memset(rdbuf, 0, sizeof(rdbuf));
	cmd = TOUCH_XSCALE_LSB;
	datalen = 8;	// 16bit x, 16bit y
	error = kuk_tr8mcu_ts_readwrite(tsdata->client,
					sizeof(cmd), &cmd,
					datalen, rdbuf);
	if (error) {
		dev_err_ratelimited(dev, "Unable to fetch data, error: %d\n",
				    error);
		goto out;
	}

	x = (rdbuf[0] | (rdbuf[1]<<8)) & 0xFFFF;
	y = (rdbuf[2] | (rdbuf[3]<<8)) & 0xFFFF;
	
	if (((x|y) & 0x8000)!=0)
	{ // invalid / pen-up
		input_mt_slot(tsdata->input, 0);
		input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, false);	// pen up
#if DEBUG_REPORTS
		dev_err(&tsdata->client->dev,"Touch UP\n");		
#endif

	}else
	{
		if( tsdata->swapxy & KUK_XNEG ) x = tsdata->xscale - x;		
		if( tsdata->swapxy & KUK_YNEG ) y = tsdata->yscale - y;
		if(  tsdata->swapxy & KUK_SWAPXY)
		{
			int t;
			t = x;
			x = y;
			y = t;
		}
		input_mt_slot(tsdata->input, 0);
		input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, true);	// pen down
		input_report_abs(tsdata->input, ABS_MT_POSITION_X,      x);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y,      y);
		
		//touchscreen_report_pos(tsdata->input, &tsdata->prop, x, y, true);
#if DEBUG_REPORTS
		dev_err(&tsdata->client->dev,"Touch [%d,%d]\n",x,y);		
#endif
	}


	if(tsdata->enable_wakeup )
	{	    
	  if(tsdata->driver_sendbuttonup!=0)
	  {
	    // printk(KERN_ERR "*** TouchScreen Send PowerButton Up ***\n");
	    input_event(tsdata->input, EV_KEY, KEY_POWER, 0);
	    tsdata->driver_sendbuttonup++;	  
	    if( tsdata->driver_suspended == 0 || tsdata->driver_sendbuttonup > 10 )
	      tsdata->driver_sendbuttonup=0;	    
	  }else
	    if( tsdata->driver_suspended)
	    {
	      // printk(KERN_ERR "*** TouchScreen IRQ in Sleep ***\n");
	      input_event(tsdata->input, EV_KEY, KEY_POWER, 1);
	      tsdata->driver_sendbuttonup=1;	    
	    }
	}
	
	input_mt_report_pointer_emulation(tsdata->input, true);
	input_sync(tsdata->input);

out:
	return IRQ_HANDLED;
}

static ssize_t kuk_tr8mcu_setting_show(struct device *dev,
				       struct device_attribute *dattr,
				       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kuk_tr8mcu_ts_data *tsdata = i2c_get_clientdata(client);
	struct kuk_tr8mcu_attribute *attr =
			container_of(dattr, struct kuk_tr8mcu_attribute, dattr);
	int val;
	size_t count = 0;
	int error = 0;

	mutex_lock(&tsdata->mutex);

	switch( attr->addr)
	{
		case T_XSCALE:
			val = tsdata->xscale;
			break;
		case T_YSCALE:
			val = tsdata->yscale;
			break;
		case T_XMIN:
			val = tsdata->xmin;
			break;
		case T_XMAX:
			val = tsdata->xmax;
			break;
		case T_YMIN:
			val = tsdata->ymin;
			break;	
		case T_YMAX:
			val = tsdata->ymax;
			break;
		case T_THRESHOLD:
			val = tsdata->threshold;
			break;
		case T_SETTLE:
			val = tsdata->settle;
			break;
		case T_SWAPXY:
			val = tsdata->swapxy;
			break;	
		case T_AVERAGE:
			val = tsdata->average;
			break;
		case T_X:
			val = kuk_tr8mcu_ts_read16(tsdata->client, TOUCH_XSCALE_LSB);
			break;
		case T_Y:
			val = kuk_tr8mcu_ts_read16(tsdata->client, TOUCH_YSCALE_LSB);
			break;
		case T_XRAW:
			val = kuk_tr8mcu_ts_read16(tsdata->client, TOUCH_XMIN_LSB);
			break;
		case T_YRAW:
			val = kuk_tr8mcu_ts_read16(tsdata->client, TOUCH_YMIN_LSB);
			break;
		case T_ZRAW:
			val = kuk_tr8mcu_ts_read16(tsdata->client, TOUCH_XMAX_Z_LSB);
			break;
		case T_ADC0:
			val = tr8_read_aux_adc(tsdata->client, 8);
			break;
		case T_ADC1:
			val = tr8_read_aux_adc(tsdata->client, 6);
			break;
		case T_ADC2:
			val = tr8_read_aux_adc(tsdata->client, 4);
			break;
		case T_ADC3:
			val = tr8_read_aux_adc(tsdata->client, 2);
			break;
		case T_TSMX:
			val = tr8_read_aux_adc(tsdata->client, 16);
			break;
		case T_TSPX:
			val = tr8_read_aux_adc(tsdata->client, 14);
			break;
		case T_TSMY:
			val = tr8_read_aux_adc(tsdata->client, 20);
			break;
		case T_TSPY:
			val = tr8_read_aux_adc(tsdata->client, 18);
			break;
		default:
			val = 0;
			break;
	}

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);

	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

static ssize_t kuk_tr8mcu_setting_store(struct device *dev,
					struct device_attribute *dattr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kuk_tr8mcu_ts_data *tsdata = i2c_get_clientdata(client);
	struct kuk_tr8mcu_attribute *attr =
			container_of(dattr, struct kuk_tr8mcu_attribute, dattr);
	unsigned int val;
	int error;

	mutex_lock(&tsdata->mutex);

	error = kstrtouint(buf, 0, &val);
	if (error)
		goto out;

	if (val < attr->limit_low || val > attr->limit_high) {
		error = -ERANGE;
		goto out;
	}


	switch( attr->addr)
	{
		case T_XSCALE:
			tsdata->xscale = val;
			kuk_tr8mcu_ts_write16(tsdata->client, TOUCH_XSCALE_LSB, tsdata->xscale );
			break;
		case T_YSCALE:
			tsdata->yscale = val;
			kuk_tr8mcu_ts_write16(tsdata->client, TOUCH_YSCALE_LSB, tsdata->yscale );
			break;
		case T_XMIN:
			tsdata->xmin = val;
			kuk_tr8mcu_ts_write16(tsdata->client, TOUCH_XMIN_LSB, tsdata->xmin );
			break;
		case T_XMAX:
			tsdata->xmax = val;
			kuk_tr8mcu_ts_write16(tsdata->client, TOUCH_XMAX_Z_LSB, tsdata->xmax );
			break;
		case T_YMIN:
			tsdata->ymin = val;
			kuk_tr8mcu_ts_write16(tsdata->client, TOUCH_YMIN_LSB, tsdata->ymin );
			break;	
		case T_YMAX:
			tsdata->ymax = val;
			kuk_tr8mcu_ts_write16(tsdata->client, TOUCH_YMAX_LSB, tsdata->ymax );
			break;
		case T_THRESHOLD:
			tsdata->threshold = val & 0x7F;
			kuk_tr8mcu_ts_write8(tsdata->client, TOUCH_CONFIG, tsdata->threshold | (tsdata->enable_touch?0x80:0x00));	// 0x80: Enable Touch, 0x40 Do Z measurement
			break;
		case T_SETTLE:
			tsdata->settle = val;
			kuk_tr8mcu_ts_write8(tsdata->client, TOUCH_WAITTIME, tsdata->settle );
			break;	
		case T_AVERAGE:
			tsdata->average = val;
			kuk_tr8mcu_ts_write8(tsdata->client, TOUCH_AVERAGE, tsdata->average );
			break;	
		case T_SWAPXY:
			tsdata->swapxy = val;
			break;
		case T_X:
			break;
		case T_Y:
			break;
		case T_XRAW:
			break;
		case T_YRAW:
			break;
		case T_ZRAW:
			break;
		case T_ADC0:
			break;
		case T_ADC1:
			break;
		case T_ADC2:
			break;
		case T_ADC3:
			break;
		case T_TSMX:
			break;
		case T_TSPX:
			break;
		case T_TSMY:
			break;
		case T_TSPY:
			break;
	}
out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

#define KUK_ATTR(_field, _mode, _addr, _limit_low, _limit_high) \
	struct kuk_tr8mcu_attribute kuk_tr8mcu_attr_##_field = {	       \
		.dattr = __ATTR(_field, _mode,			               \
				kuk_tr8mcu_setting_show,		       \
				kuk_tr8mcu_setting_store),		       \
		.addr = _addr,                                                 \
		.limit_low = _limit_low,				       \
		.limit_high = _limit_high				       \
	}

static KUK_ATTR( xscale		, S_IWUSR | S_IRUGO	, T_XSCALE		, 0	, 4095);
static KUK_ATTR( yscale		, S_IWUSR | S_IRUGO	, T_YSCALE		, 0	, 4095);
static KUK_ATTR( xmin		, S_IWUSR | S_IRUGO	, T_XMIN		, 0	, 0xFFFF);
static KUK_ATTR( xmax		, S_IWUSR | S_IRUGO	, T_XMAX		, 0	, 0xFFFF);
static KUK_ATTR( ymin		, S_IWUSR | S_IRUGO	, T_YMIN		, 0	, 0xFFFF);
static KUK_ATTR( ymax		, S_IWUSR | S_IRUGO	, T_YMAX		, 0	, 0xFFFF);
static KUK_ATTR( threshold	, S_IWUSR | S_IRUGO	, T_THRESHOLD		, 0	, 64);
static KUK_ATTR( settle		, S_IWUSR | S_IRUGO	, T_SETTLE		, 0	, 255);
static KUK_ATTR( average	, S_IWUSR | S_IRUGO	, T_AVERAGE		, 0	, 0xFFFF);
static KUK_ATTR( swapxy		, S_IWUSR | S_IRUGO	, T_SWAPXY		, 0	, 16);
static KUK_ATTR( x		, S_IWUSR | S_IRUGO	, T_X			, 0	, 0);
static KUK_ATTR( y		, S_IWUSR | S_IRUGO	, T_Y			, 0	, 0);
static KUK_ATTR( xraw		, S_IWUSR | S_IRUGO	, T_XRAW		, 0	, 0);
static KUK_ATTR( yraw		, S_IWUSR | S_IRUGO	, T_YRAW		, 0	, 0);
static KUK_ATTR( zraw		, S_IWUSR | S_IRUGO	, T_ZRAW		, 0	, 0);
static KUK_ATTR( adc0		, S_IWUSR | S_IRUGO	, T_ADC0		, 0	, 0);
static KUK_ATTR( adc1		, S_IWUSR | S_IRUGO	, T_ADC1		, 0	, 0);
static KUK_ATTR( adc2		, S_IWUSR | S_IRUGO	, T_ADC2		, 0	, 0);
static KUK_ATTR( adc3		, S_IWUSR | S_IRUGO	, T_ADC3		, 0	, 0);
static KUK_ATTR( tsmx		, S_IWUSR | S_IRUGO	, T_TSMX		, 0	, 0);
static KUK_ATTR( tspx		, S_IWUSR | S_IRUGO	, T_TSPX		, 0	, 0);
static KUK_ATTR( tsmy		, S_IWUSR | S_IRUGO	, T_TSMY		, 0	, 0);
static KUK_ATTR( tspy		, S_IWUSR | S_IRUGO	, T_TSPY		, 0	, 0);


static struct attribute *kuk_tr8mcu_attrs[] = 
{
	&kuk_tr8mcu_attr_xscale.dattr.attr,
	&kuk_tr8mcu_attr_yscale.dattr.attr,
	&kuk_tr8mcu_attr_xmin.dattr.attr,
	&kuk_tr8mcu_attr_xmax.dattr.attr,
	&kuk_tr8mcu_attr_ymin.dattr.attr,
	&kuk_tr8mcu_attr_ymax.dattr.attr,
	&kuk_tr8mcu_attr_threshold.dattr.attr,
	&kuk_tr8mcu_attr_settle.dattr.attr,
	&kuk_tr8mcu_attr_average.dattr.attr,
	&kuk_tr8mcu_attr_swapxy.dattr.attr,
	&kuk_tr8mcu_attr_x.dattr.attr,
	&kuk_tr8mcu_attr_y.dattr.attr,
	&kuk_tr8mcu_attr_xraw.dattr.attr,
	&kuk_tr8mcu_attr_yraw.dattr.attr,
	&kuk_tr8mcu_attr_zraw.dattr.attr,
	&kuk_tr8mcu_attr_adc0.dattr.attr,
	&kuk_tr8mcu_attr_adc1.dattr.attr,
	&kuk_tr8mcu_attr_adc2.dattr.attr,
	&kuk_tr8mcu_attr_adc3.dattr.attr,
	&kuk_tr8mcu_attr_tsmx.dattr.attr,
	&kuk_tr8mcu_attr_tspx.dattr.attr,
	&kuk_tr8mcu_attr_tsmy.dattr.attr,
	&kuk_tr8mcu_attr_tspy.dattr.attr,
	NULL
};

static const struct attribute_group kuk_tr8mcu_attr_group = {
	.attrs = kuk_tr8mcu_attrs,
};


#ifdef CONFIG_OF
static int kuk_tr8mcu_i2c_ts_probe_dt(struct device *dev,
				struct kuk_tr8mcu_ts_data *tsdata)
{
	struct device_node *np = dev->of_node;
	const char *prop;
	u32 val;
	/*
	 * irq_pin is not needed for DT setup.
	 * irq is associated via 'interrupts' property in DT
	 */
	tsdata->reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
	tsdata->disable   = 0;
	
	prop = of_get_property(np,    "swapxy", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->swapxy |= KUK_SWAPXY;	  

	prop = of_get_property(np,    "xreverse", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->swapxy |= KUK_XNEG;	  

	prop = of_get_property(np,    "yreverse", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->swapxy |= KUK_YNEG;	  

	prop = of_get_property(np,    "enable_wakeup", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->enable_wakeup = 1;
	else		                                    tsdata->enable_wakeup = 0;

	prop = of_get_property(np,    "enable_touch", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->enable_touch = 1;
	else		                                    tsdata->enable_touch = 0;

	if ( of_property_read_u32(np, "xscale"	, &val	) == 0)
		tsdata->xscale = (u16)val;
	if ( of_property_read_u32(np, "yscale"	, &val	) == 0)
		tsdata->yscale = (u16)val;
	if ( of_property_read_u32(np, "xmin"	, &val	) == 0)
		tsdata->xmin = (u16)val;
	if ( of_property_read_u32(np, "xmax"	, &val	) == 0)
		tsdata->xmax = (u16)val;
	if ( of_property_read_u32(np, "ymin"	, &val	) == 0)
		tsdata->ymin = (u16)val;
	if ( of_property_read_u32(np, "ymax"	, &val	) == 0)
		tsdata->ymax = (u16)val;
	if ( of_property_read_u32(np, "threshold", &val	) == 0)
		tsdata->threshold = (u8)val;
	if ( of_property_read_u32(np, "settle"	, &val	) == 0)
		tsdata->settle = (u8)val;
	if ( of_property_read_u32(np, "average"	, &val	) == 0)
		tsdata->average = (u8)val;
	if ( of_property_read_u32(np,"touchdisable",&val) == 0)
		tsdata->disable = (u8)val;

	return 0;
}
#else
static inline int kuk_tr8mcu_i2c_ts_probe_dt(struct device *dev,
					struct kuk_tr8mcu_ts_data *tsdata)
{
	return -ENODEV;
}
#endif

extern int handle_reserved_delayed(void);
extern int handle_reserved_mcu(void);

static int probe_mcu_gpio(struct i2c_client          *client,
			  struct kuk_tr8mcu_ts_data  *tsdata)
{
  struct mcu_gpio *gpio;
  int		   status;

  printk(KERN_ERR "probe_mcu_gpio *********************************\n");
  
  gpio = &tsdata->tr8mcugpio;

  mutex_init(&gpio->lock);
  gpio->chip.can_sleep		= true;
  gpio->chip.parent		= &client->dev;
  gpio->chip.label              = client->name;
  gpio->chip.owner		= THIS_MODULE;
  gpio->chip.get		= kuk_tr8mcu_gpio_read;
  gpio->chip.set		= kuk_tr8mcu_gpio_write;
  gpio->chip.direction_input	= kuk_tr8mcu_gpio_direction_input;
  gpio->chip.direction_output	= kuk_tr8mcu_gpio_direction_output;
  gpio->chip.base		= 320;
  gpio->chip.ngpio		= 11;
  gpio->chip.request            = NULL; // kuk_tr8mcu_gpio_request;
  gpio->client                  = client;
  if(  client->adapter )
  {
    gpio->client->adapter = client->adapter;    
  }else
  {      
    printk("KERN_ERR *** probe_mcu_gpio No client->adapter \n");
  }
  //  i2c_set_clientdata(client, tsdata);

  status = devm_gpiochip_add_data(&gpio->client->dev, &gpio->chip, gpio);
  //status = devm_gpiochip_add_data(&client->dev,       &gpio->chip, gpio);

  if (status < 0)
    goto fail;

  printk(KERN_ERR "probe_mcu_gpio success !***\n");
  handle_reserved_mcu();  
  return(0);
 

fail:
  dev_dbg(&client->dev, "probe_mcu_gpio error %d for '%s'\n", status,
		client->name);

	return status;
  
}

DEFINE_MUTEX( sync_kuk_tr8mcu_config1_mutex);
static int mcu_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct kuk_tr8mcu_ts_data *tsdata = rdev_get_drvdata(rdev);

	if ( tsdata->reg_config1 & MCU_REG_CONFIG1_SDVSEL)
		return 1800000;
	else
		return 3300000;

}
static int mcu_regulator_set_voltage(struct regulator_dev *rdev,
				     int req_min_uV, int req_max_uV,
				     unsigned int *selector)
{
	struct kuk_tr8mcu_ts_data *tsdata = rdev_get_drvdata(rdev);
	int new_config;

	mutex_lock(&sync_kuk_tr8mcu_config1_mutex);
	new_config = tsdata->reg_config1;
	if (( req_min_uV <= 1800000)&&( 1800000 <= req_max_uV))
	{
		new_config |= MCU_REG_CONFIG1_SDVSEL;
		dev_dbg(&tsdata->client->dev, "[MCU] Set SDCard voltage to 1.8V.\n");
	}else
	if (( req_min_uV <= 3300000)&&( 3300000 <= req_max_uV))
	{
		new_config &= ~MCU_REG_CONFIG1_SDVSEL;
		dev_dbg(&tsdata->client->dev, "[MCU] Set SDCard voltage to 3.3V.\n");
	}else
	{
		mutex_unlock(&sync_kuk_tr8mcu_config1_mutex);
		dev_err(&tsdata->client->dev, "[MCU] Set Voltage unsupported %d..%dmV\n", req_min_uV, req_max_uV);
		return -EINVAL;
	}
	if ( new_config != tsdata->reg_config1)
	{
		tsdata->reg_config1 = new_config;
		dev_dbg(&tsdata->client->dev, "[MCU] Set MCU_REG_CONFIG1=0x%x\n", tsdata->reg_config1);
		kuk_tr8mcu_ts_write8( tsdata->client, MCU_REG_CONFIG1, tsdata->reg_config1);
	}
	mutex_unlock(&sync_kuk_tr8mcu_config1_mutex);

	return 0;
}
static int mcu_regulator_enable(struct regulator_dev *dev)
{
	return 0;
}

static int mcu_regulator_disable(struct regulator_dev *dev)
{
	return 0;
}

static int mcu_regulator_is_enabled(struct regulator_dev *dev)
{
	return true;
}

static const struct regulator_ops mcu_regulator_ops = {
	.get_voltage = mcu_regulator_get_voltage,
	.set_voltage = mcu_regulator_set_voltage,
	.enable      = mcu_regulator_enable,
	.disable     = mcu_regulator_disable,
	.is_enabled  = mcu_regulator_is_enabled,
	.map_voltage = regulator_map_voltage_linear,
	.list_voltage = regulator_list_voltage_linear,
};
static int probe_mcu_regulator(struct i2c_client          *client,
			  struct kuk_tr8mcu_ts_data  *tsdata)
{
	struct regulator_desc *rdesc = &tsdata->mcu_regulator_desc;
	struct regulator_config cfg = { };
	const struct regulator_init_data *init_data;

	dev_dbg(&client->dev, "MCU probe regulator\n");
	rdesc->name = "mcu-reg";
	rdesc->supply_name = "vdd_sdcard";
	rdesc->ops = &mcu_regulator_ops;
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->n_voltages = 2;
	rdesc->min_uV = 1800000;
	rdesc->uV_step = (3300000-1800000);
	rdesc->owner = THIS_MODULE;

	init_data = of_get_regulator_init_data(&client->dev, client->dev.of_node, rdesc);
	cfg.dev = &client->dev;
	cfg.init_data = init_data;
	cfg.driver_data = tsdata;
	cfg.of_node = client->dev.of_node;

	tsdata->reg_config1 = kuk_tr8mcu_ts_read8(tsdata->client, MCU_REG_CONFIG1);
	tsdata->mcu_regulator_dev = devm_regulator_register(&client->dev, rdesc, &cfg);
	return PTR_ERR_OR_ZERO(tsdata->mcu_regulator_dev);	
}

struct i2c_client *tr8_adc_get_handle(void)
{
  return(NULL);  
}

DEFINE_MUTEX( sync_kuk_tr8mcu_adc_mutex);

struct i2c_client         *tr8_i2c_client=NULL;
struct kuk_tr8mcu_ts_data *tr8_ts_data=NULL;
struct device             *tr8_mcu_dev=NULL;

struct i2c_client *tr8_adc_get_i2c_client(void)
{
  return(tr8_i2c_client);  
}


 
int tr8_read_aux_adc(struct i2c_client *client, int adcsel)
{
	int data;
	int tries = 5;
	
	mutex_lock(&sync_kuk_tr8mcu_adc_mutex);
	{
		kuk_tr8mcu_ts_write8( client, MCU_REG_ADC, adcsel);
		while ( tries--)
		{
			data = kuk_tr8mcu_ts_read24( client, MCU_REG_ADC);
			if (( data&0xFF) == adcsel)
				break;
		}
	}
	mutex_unlock(&sync_kuk_tr8mcu_adc_mutex);

	if (( tries <= 0)||((data&0xFF) != adcsel))
	{
		dev_err(&client->dev, "failed to read adc(%d) %d,0x%x.\n", adcsel, tries, data);
		return -1;
	}

	dev_dbg(&client->dev, "adc%d: %d (tries %d; scaled:%dmV)\n", adcsel, ((data>>8)&0xFFFF), tries, (((data>>8)&0xFFFF)*3300*2)/4096);
	return ((data>>8)&0xFFFF);
}

int tr8_read_adc(int adcsel)
{
    struct i2c_client *client;

    if(tr8_mcu_dev==NULL)     return -1;

    client = to_i2c_client(tr8_mcu_dev);

    if(client==NULL)     return -1;

    return(tr8_read_aux_adc(client,adcsel));    
}


static int kuk_tr8mcu_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	struct kuk_tr8mcu_ts_data *tsdata;
	struct input_dev *input;
	int error;
	
	dev_dbg(&client->dev, "probing for KUK TR8MCU Touch I2C\n");

	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}


	strcpy(tsdata->name,  "TR8MCU_TOUCH");
	tsdata->irq_pin   = -EINVAL;
	tsdata->swapxy    	= 0;
	
	tsdata->xscale		= 800;
	tsdata->yscale		= 480;
	tsdata->xmin 		= 0;
	tsdata->xmax		= 4095;
	tsdata->ymin		= 0;
	tsdata->ymax		= 4095;
	tsdata->threshold	= 32;
	tsdata->settle		= 3;
	tsdata->average		= 2;


	error = kuk_tr8mcu_i2c_ts_probe_dt(&client->dev, tsdata);
	if (error) {
		dev_err(&client->dev,
			"DT probe failed and no platform data present\n");
		return error;
	}

		
	if (gpio_is_valid(tsdata->irq_pin)) {
		error = devm_gpio_request_one(&client->dev, tsdata->irq_pin,
					GPIOF_IN, "kuk-tr8mcu irq");
	if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				tsdata->irq_pin, error);
			return error;
		}
	}

	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	mutex_init(&tsdata->mutex);
	tsdata->client = client;
	tsdata->input = input;
	tsdata->driver_suspended    = 0;
	tsdata->driver_sendbuttonup = 0;
	

	printk( KERN_ERR "TR8MCU_TOUCH : swapxy=0x%x enable_wakeup=%d irq_pin=0x%x reset_pin=0x%x\n",
		tsdata->swapxy, tsdata->enable_wakeup, tsdata->irq_pin, tsdata->reset_pin);


	kuk_tr8mcu_ts_write16(tsdata->client , TOUCH_XSCALE_LSB		, tsdata->xscale);
	kuk_tr8mcu_ts_write16(tsdata->client , TOUCH_YSCALE_LSB		, tsdata->yscale);
	kuk_tr8mcu_ts_write16(tsdata->client , TOUCH_XMIN_LSB		, tsdata->xmin);
	kuk_tr8mcu_ts_write16(tsdata->client , TOUCH_XMAX_Z_LSB		, tsdata->xmax);
	kuk_tr8mcu_ts_write16(tsdata->client , TOUCH_YMIN_LSB		, tsdata->ymin);
	kuk_tr8mcu_ts_write16(tsdata->client , TOUCH_YMAX_LSB		, tsdata->ymax);
	kuk_tr8mcu_ts_write8(tsdata->client , TOUCH_CONFIG		, tsdata->threshold);
	kuk_tr8mcu_ts_write8(tsdata->client , TOUCH_WAITTIME		, tsdata->settle);
	kuk_tr8mcu_ts_write8(tsdata->client , TOUCH_AVERAGE		, tsdata->average);

	input->name = tsdata->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_SYN,    input->evbit);
	__set_bit(EV_KEY,    input->evbit);
	__set_bit(EV_ABS,    input->evbit);
	__set_bit(EV_REP,    input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(KEY_POWER, input->keybit);

	if(  tsdata->swapxy & KUK_SWAPXY)	    
	{
		input_set_abs_params(input, ABS_Y,             0, tsdata->xscale, 0, 0);
		input_set_abs_params(input, ABS_X,             0, tsdata->yscale, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, tsdata->xscale, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, tsdata->yscale, 0, 0);
	}else
	{
		input_set_abs_params(input, ABS_X,             0, tsdata->xscale, 0, 0);
		input_set_abs_params(input, ABS_Y,             0, tsdata->yscale, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, tsdata->xscale, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, tsdata->yscale, 0, 0);
	}

	touchscreen_parse_properties(input, true, &tsdata->prop);

	error = input_mt_init_slots(input, MAX_SUPPORT_POINTS, 0);

	if (error) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		return error;
	}

	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	if(  tsdata->disable == 0 )
	{	    

	  kuk_tr8mcu_ts_write16(tsdata->client , TOUCH_CONFIG, tsdata->threshold | 0x80);	// Enable Touch!
	
	  error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					kuk_tr8mcu_ts_isr,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, tsdata);
	  if (error) {
	    dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
	    return error;
	  }
	}
	
	error = sysfs_create_group(&client->dev.kobj, &kuk_tr8mcu_attr_group);
	if (error)
		return error;

	error = input_register_device(input);
	if (error)
		goto err_remove_attrs;

	device_init_wakeup(&client->dev, 1);
	
	dev_err(&client->dev,
		"KUK TR8MCU Touch initialized: IRQ %d, Reset Gpio %d. Probe OK\n",
		client->irq, tsdata->reset_pin);

	tr8_i2c_client = tsdata->client;
	tr8_ts_data    = tsdata;
	tr8_mcu_dev    = &client->dev;
#if 0
	dev_err(&client->dev, "KUK TR8MCU ADC0 = %d\n ", tr8_read_adc(  8 ));	
	dev_err(&client->dev, "KUK TR8MCU ADC1 = %d\n ", tr8_read_adc(  6 ));	
	dev_err(&client->dev, "KUK TR8MCU ADC2 = %d\n ", tr8_read_adc(  4 ));       
	dev_err(&client->dev, "KUK TR8MCU ADC3 = %d\n ", tr8_read_adc(  2 ));	
#endif	
	probe_mcu_gpio(client,tsdata);
	probe_mcu_regulator( client, tsdata);

	return 0;

err_remove_attrs:
	sysfs_remove_group(&client->dev.kobj, &kuk_tr8mcu_attr_group);
	return error;
}

static int kuk_tr8mcu_ts_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &kuk_tr8mcu_attr_group);

	return 0;
}

static int __maybe_unused kuk_tr8mcu_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kuk_tr8mcu_ts_data *tsdata = i2c_get_clientdata(client);

	tsdata->driver_suspended++;	
	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused kuk_tr8mcu_ts_resume(struct device *dev)
{
	struct i2c_client *client         = to_i2c_client(dev);
	struct kuk_tr8mcu_ts_data *tsdata = i2c_get_clientdata(client);
	tsdata->driver_suspended--;	
	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(kuk_tr8mcu_ts_pm_ops,
			 kuk_tr8mcu_ts_suspend, kuk_tr8mcu_ts_resume);

static const struct i2c_device_id kuk_tr8mcu_ts_id[] = {
	{ "kuk-tr8mcu-touch", 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, kuk_tr8mcu_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id kuk_tr8mcu_of_match[] = {
	{ .compatible = "kuk,tr8mcu-touch", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, kuk_tr8mcu_of_match);
#endif

static struct i2c_driver kuk_tr8mcu_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "kuk_tr8mcu_touch",
		.of_match_table = of_match_ptr(kuk_tr8mcu_of_match),
		.pm = &kuk_tr8mcu_ts_pm_ops,
	},
	.id_table = kuk_tr8mcu_ts_id,
	.probe    = kuk_tr8mcu_ts_probe,
	.remove   = kuk_tr8mcu_ts_remove,
};

module_i2c_driver(kuk_tr8mcu_ts_driver);

MODULE_AUTHOR("Sven Hillger <hillger@keith-koep.com>");
MODULE_DESCRIPTION("KUK TR8MCU I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
