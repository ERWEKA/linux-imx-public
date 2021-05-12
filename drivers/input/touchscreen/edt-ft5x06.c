/*
 * Copyright (C) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 * Daniel Wagener <daniel.wagener@kernelconcepts.de> (M09 firmware support)
 * Lothar Waßmann <LW@KARO-electronics.de> (DT support)
 *
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
 * This is a driver for the EDT "Polytouch" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 * Development of this driver has been sponsored by Glyn:
 *    http://www.glyn.com/Products/Displays
 */

#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/input/edt-ft5x06.h>
#define DEBUG_REPORTS                      0
#define MAX_SUPPORT_POINTS		   5

#define WORK_REGISTER_THRESHOLD		0x00
#define WORK_REGISTER_REPORT_RATE	0x08
#define WORK_REGISTER_GAIN		0x30
#define WORK_REGISTER_OFFSET		0x31
#define WORK_REGISTER_NUM_X		0x33
#define WORK_REGISTER_NUM_Y		0x34
#define NO_REGISTER			0xff

#define M09_REGISTER_THRESHOLD          0x80
#define M09_REGISTER_REPORT_RATE        0x00
#define M09_REGISTER_GAIN               0x92
#define M09_REGISTER_OFFSET             0x93
#define M09_REGISTER_NUM_X              0x94
#define M09_REGISTER_NUM_Y              0x95

#define FT5316DME_REGISTER_GAIN         0x07
#define FT5316DME_SWITCH_MODE_DELAY     125

#define WORK_REGISTER_OPMODE		0x3c
#define FACTORY_REGISTER_OPMODE		0x01

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED		0x03

#define EDT_NAME_LEN			23
#define EDT_SWITCH_MODE_RETRIES		10
#define EDT_SWITCH_MODE_DELAY		5 /* msec */
#define EDT_RAW_DATA_RETRIES		100
#define EDT_RAW_DATA_DELAY		1 /* msec */

#define SaveU16Get( x)                  (((((PUCHAR)(&(x)))[1])|((((PUCHAR)(&(x)))[0])<<8))&0xFFFF)

enum edt_ver {
    UNKNOWN_AUTODETECT = 0,
	M06,
	M09,
	EV,
    FT5316DME,
};

static int swapxy=0;
static int xneg=0;
static int yneg=0;
static int maxx=0;
static int maxy=0;


struct edt_reg_addr {
	int reg_threshold;
	int reg_report_rate;
	int reg_gain;
	int reg_offset;
	int reg_num_x;
	int reg_num_y;
};

struct edt_ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	u16 num_x;
	u16 num_y;

	int reset_pin;
	int enable_wakeup;
	int irq_pin;
	int wake_pin;
        int swapxy;
	int prop_num_x;
	int prop_num_y;
	int prop_xres;
	int prop_yres;

#if defined(CONFIG_DEBUG_FS)
	struct dentry *debug_dir;
	u8 *raw_buffer;
	size_t raw_bufsize;
#endif

	struct mutex mutex;
	bool factory_mode;
	int threshold;
	int gain;
	int offset;
	int report_rate;

	char name[EDT_NAME_LEN];

	struct edt_reg_addr reg_addr;
	enum edt_ver version;
    enum edt_ver alternate_version;
        int driver_suspended;
        int driver_sendbuttonup;  
};

#define EDT_VERS_STR( version )  (((version)==M09)?"M09":(((version)==M06)?"M06":"EV"))

static int edt_ft5x06_ts_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

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

static bool edt_ft5x06_ts_check_crc(struct edt_ft5x06_ts_data *tsdata,
				    u8 *buf, int buflen)
{
	int i;
	u8 crc = 0;

	for (i = 0; i < buflen - 1; i++)
		crc ^= buf[i];

	if (crc != buf[buflen-1]) {
		dev_err_ratelimited(&tsdata->client->dev,
				    "crc error: 0x%02x expected, got 0x%02x\n",
				    crc, buf[buflen-1]);
		return false;
	}

	return true;
}
#ifdef CONFIG_MXC_MMA8451_xxx
extern int mxc_mma8451_sensor_enable;
#endif

int PlatformIsSuspended(struct edt_ft5x06_ts_data *tsdata)
{
  if( tsdata->driver_suspended      ) return(1);
#ifdef CONFIG_MXC_MMA8451_xxx
  if( mxc_mma8451_sensor_enable=='0') return(1);
#endif
  return(0);  
}

//static int SuspendStatus=0;

/*
 * Touch Protocol:
 * on command, the controller sends a data load with a predefined length
 * with a predefined structure
 */
static irqreturn_t edt_ft5x06_ts_isr(int irq, void *dev_id)
{
	struct edt_ft5x06_ts_data *tsdata = dev_id;
	struct device *dev = &tsdata->client->dev;
	int i, type, x, y, id, offset, tplen, datalen;
	int error;
	u8 cmd = 0xf9;
	u8 rdbuf[29];
 
	// printk(KERN_ERR "*** TouchScreen IRQ ***\n");

	switch (tsdata->version){
		case M06:{
			cmd = 0xf9; //tell the controller to send touch data
			offset = 5; //where the actual touch data starts
			tplen = 4; //data comes in so called frames
			datalen = 26; //how much bytes to listen for
			break;
		}
        case FT5316DME:
	    case EV:
		case M09:{
			cmd = 0x02;
			offset = 1;
			tplen = 6;
			datalen = 29;
			break;
		}
		default:
			goto out;
	}

	memset(rdbuf, 0, sizeof(rdbuf));

	error = edt_ft5x06_ts_readwrite(tsdata->client,
					sizeof(cmd), &cmd,
					datalen, rdbuf);
	if (error) {
		dev_err_ratelimited(dev, "Unable to fetch data, error: %d\n",
				    error);
		goto out;
	}

	/* M09 does not send header or CRC */
	if (tsdata->version == M06) {
		if (rdbuf[0] != 0xaa || rdbuf[1] != 0xaa ||
			rdbuf[2] != datalen) {
			dev_err_ratelimited(dev,
					"Unexpected header: %02x%02x%02x!\n",
					rdbuf[0], rdbuf[1], rdbuf[2]);
			goto out;
		}

		if (!edt_ft5x06_ts_check_crc(tsdata, rdbuf, datalen))
			goto out;
	}

	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
		u8 *buf = &rdbuf[i * tplen + offset];
		bool down;

		type = buf[0] >> 6;
		/* ignore Reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		/* M06 sometimes sends bogus coordinates in TOUCH_DOWN */
		if (tsdata->version == M06 && type == TOUCH_EVENT_DOWN)
			continue;

		x = ((buf[0] << 8) | buf[1]) & 0x0fff;
		y = ((buf[2] << 8) | buf[3]) & 0x0fff;

		if ((i==0)&&(tsdata->version == EV))
	        {		    
		  if( (x ==     0) || (y ==    0)
                   || (x == 0x7FF) || (y = 0x7FF)
		   || (rdbuf[2] > 2) ) // Number of Fingers
		  {
		    //printk( KERN_ERR "EDT Ignore Frame %d,%d  %d\n",x,y,rdbuf[2]);				      
		    goto out;
		  }
		}
		
		id = (buf[2] >> 4) & 0x0f;
		down = (type != TOUCH_EVENT_UP);

		if(  swapxy )
		{
		  if( yneg ) y = maxx - y;
		  if( xneg ) x = maxy - x;		
#if DEBUG_REPORTS
		  dev_err(&tsdata->client->dev,"*** TouchScreen %d x:%d y:%d id:%d type:%d mx%d,my%d\n",
			  i,y,x,id,type,maxx, maxy);		
#endif
		}else
		{		    
		  if( yneg ) y = maxy - y;
		  if( xneg ) x = maxx - x;		
#if DEBUG_REPORTS
		  dev_err(&tsdata->client->dev,"*** TouchScreen %d x:%d y:%d id:%d type:%d mx%d,my%d\n",
			  i,x,y,id,type,maxx,maxy);		
#endif
		}
		//printk( KERN_ERR "Report ID %d Finger %d\n",id,down);		
		input_mt_slot(tsdata->input, id);
		input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, down);

		if (!down)
			continue;

		if( swapxy )
		{
		  input_report_abs(tsdata->input, ABS_MT_POSITION_Y,      x);
		  input_report_abs(tsdata->input, ABS_MT_POSITION_X,      y);
		}else
		{
		  input_report_abs(tsdata->input, ABS_MT_POSITION_X,      x);
		  input_report_abs(tsdata->input, ABS_MT_POSITION_Y,      y);
		}
		//printk( KERN_ERR "ABS_MT_POSITION_X %d ABS_MT_POSITION_Y %d\n",x,y);
		
	}
	if(tsdata->enable_wakeup )
	{	    
	  if(tsdata->driver_sendbuttonup!=0)
	  {
	    // printk(KERN_ERR "*** TouchScreen Send PowerButton Up ***\n");
	    input_event(tsdata->input, EV_KEY, KEY_POWER, 0);
	    tsdata->driver_sendbuttonup++;	  
	    if( PlatformIsSuspended(tsdata)==0 || tsdata->driver_sendbuttonup > 10 )
	      tsdata->driver_sendbuttonup=0;	    
	  }else
	    if( PlatformIsSuspended(tsdata))
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

static int edt_ft5x06_register_write(struct edt_ft5x06_ts_data *tsdata,
				     u8 addr, u8 value)
{
	switch (tsdata->version){
		case M06:{
			u8 wrbuf[4];

			wrbuf[0] = tsdata->factory_mode	? 0xf3 : 0xfc;
			wrbuf[1] = tsdata->factory_mode	? addr & 0x7f : addr & 0x3f;
			wrbuf[2] = value;
			wrbuf[3] = wrbuf[0] ^ wrbuf[1] ^ wrbuf[2];

			return edt_ft5x06_ts_readwrite(tsdata->client, 4, wrbuf, 0, NULL);
		}
        case FT5316DME:
	    case EV:	
		case M09:{
			u8 wrbuf[2];
			wrbuf[0] = addr;
			wrbuf[1] = value;
			return edt_ft5x06_ts_readwrite(tsdata->client, 2, wrbuf, 0, NULL);
		}
		default:
			return -EINVAL;
	}
}

static int edt_ft5x06_register_read(struct edt_ft5x06_ts_data *tsdata,
				    u8 addr)
{
	int error;
	switch (tsdata->version)
	{
	   case M06:{
			u8 wrbuf[2], rdbuf[2];
			wrbuf[0] = tsdata->factory_mode  ? 0xf3 : 0xfc;
			wrbuf[1] = tsdata->factory_mode  ? addr & 0x7f : addr & 0x3f;
			wrbuf[1] |= tsdata->factory_mode ? 0x80 : 0x40;

			error = edt_ft5x06_ts_readwrite(tsdata->client,	2, wrbuf, 2, rdbuf);
				return error;

			if ((wrbuf[0] ^ wrbuf[1] ^ rdbuf[0]) != rdbuf[1]) {
				dev_err(&tsdata->client->dev,
						"crc error: 0x%02x expected, got 0x%02x\n",
						wrbuf[0] ^ wrbuf[1] ^ rdbuf[0],	rdbuf[1]);
				return -EIO;
			}
			return rdbuf[0];
	   }
       case FT5316DME:
	   case EV:
	   case M09:{
			u8 wrbuf[0], rdbuf[0];
			wrbuf[0] = addr;
			error = edt_ft5x06_ts_readwrite(tsdata->client, 1, wrbuf, 1, rdbuf);
			if (error)
				return error;
			return rdbuf[0];
		}
		default:
			return -EINVAL;
	}
	return 0;
}

struct edt_ft5x06_attribute {
	struct device_attribute dattr;
	size_t field_offset;
	u8 limit_low;
	u8 limit_high;
	u8 addr_m06;
	u8 addr_m09;
    u8 addr_ft5316;
};

#define EDT_ATTR(_field, _mode, _addr_m06, _addr_m09, _addr_ft5316, _limit_low, _limit_high) \
	struct edt_ft5x06_attribute edt_ft5x06_attr_##_field = {	       \
		.dattr = __ATTR(_field, _mode,			               \
				edt_ft5x06_setting_show,		       \
				edt_ft5x06_setting_store),		       \
		.field_offset =						       \
			offsetof(struct edt_ft5x06_ts_data, _field),	       \
		.addr_m06 = _addr_m06,					       \
		.addr_m09 = _addr_m09,					       \
        .addr_ft5316 = _addr_ft5316,                    \
		.limit_low = _limit_low,				       \
		.limit_high = _limit_high,				       \
	}

static ssize_t edt_ft5x06_setting_show(struct device *dev,
				       struct device_attribute *dattr,
				       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)((char *)tsdata + attr->field_offset);
	int val;
	size_t count = 0;
	int error = 0;
	u8 addr;

	mutex_lock(&tsdata->mutex);

	if (tsdata->factory_mode) {
		error = -EIO;
		goto out;
	}
	switch (tsdata->version){
		case M06:
			addr = attr->addr_m06;
			break;
        case FT5316DME:
            addr = attr->addr_ft5316;
            break;
		case EV:
		case M09:
			addr = attr->addr_m09;
			break;
		default:
			addr = 0;
			break;
	}

	if (addr != NO_REGISTER) {
	    if (tsdata->version == FT5316DME)
        {
            edt_ft5x06_register_write(tsdata,  0, 0x40);    // Set to Test-Mode
		    val = edt_ft5x06_register_read(tsdata, addr);
            mdelay(FT5316DME_SWITCH_MODE_DELAY);
            edt_ft5x06_register_write(tsdata,  0, 0x00);    // Set to Operating-Mode
        }else            
        {
		    val = edt_ft5x06_register_read(tsdata, addr);
        }
	        // printk( KERN_ERR "EDT-(%s) Red Reg 0x%x  %d\n",EDT_VERS_STR(tsdata->version), addr, val);
		if (val < 0) {
			error = val;
			dev_err(&tsdata->client->dev,
				"Failed to fetch attribute %s, error %d\n",
				dattr->attr.name, error);
			goto out;
		}
	} else {
		val = *field;
	}

	if (val != *field) {
		dev_warn(&tsdata->client->dev,
			 "%s: read (%d) and stored value (%d) differ\n",
			 dattr->attr.name, val, *field);
		*field = val;
	}

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);
out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

static ssize_t edt_ft5x06_setting_store(struct device *dev,
					struct device_attribute *dattr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)((char *)tsdata + attr->field_offset);
	unsigned int val;
	int error;
	u8 addr = 0;

	mutex_lock(&tsdata->mutex);

	if (tsdata->factory_mode) {
		error = -EIO;
		goto out;
	}

	error = kstrtouint(buf, 0, &val);
	if (error)
		goto out;

	if (val < attr->limit_low || val > attr->limit_high) {
		error = -ERANGE;
		goto out;
	}

	switch (tsdata->version){
		case M06:
			addr = attr->addr_m06;
			break;
		case EV:
		case M09:
			addr = attr->addr_m09;
			break;
        case FT5316DME:
            addr = attr->addr_ft5316;
            break;
		default:
			addr = 0;
			break;
	}

	if (addr != NO_REGISTER) 
	{
	        // unsigned int readback;
	    if (tsdata->version == FT5316DME)
        {
            edt_ft5x06_register_write(tsdata,  0, 0x40);    // Set to Test-Mode
		    error = edt_ft5x06_register_write(tsdata, addr, val);
            mdelay(FT5316DME_SWITCH_MODE_DELAY);
            edt_ft5x06_register_write(tsdata,  0, 0x00);    // Set to Operating-Mode
        }else
        {            
		    error = edt_ft5x06_register_write(tsdata, addr, val);
        }
		// readback = edt_ft5x06_register_read(tsdata, addr);
	        // printk( KERN_ERR "EDT Wrote-(%s) %d to Reg 0x%x readback %d\n",EDT_VERS_STR(tsdata->version),val,addr,readback);
		if (error) {
			dev_err(&tsdata->client->dev,
				"Failed to update attribute %s, error: %d\n",
				dattr->attr.name, error);
			goto out;
		}
	}
	*field = val;

out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

static EDT_ATTR(gain, S_IWUSR        | S_IRUGO, WORK_REGISTER_GAIN,	   M09_REGISTER_GAIN, FT5316DME_REGISTER_GAIN, 0, 31);
static EDT_ATTR(offset, S_IWUSR      | S_IRUGO, WORK_REGISTER_OFFSET,      M09_REGISTER_OFFSET, NO_REGISTER,       0, 31);
static EDT_ATTR(threshold, S_IWUSR   | S_IRUGO,	WORK_REGISTER_THRESHOLD,   M09_REGISTER_THRESHOLD, NO_REGISTER, 20, 80);
static EDT_ATTR(report_rate, S_IWUSR | S_IRUGO,	WORK_REGISTER_REPORT_RATE, NO_REGISTER, NO_REGISTER, 3, 14);

static struct attribute *edt_ft5x06_attrs[] = 
{
	&edt_ft5x06_attr_gain.dattr.attr,
	&edt_ft5x06_attr_offset.dattr.attr,
	&edt_ft5x06_attr_threshold.dattr.attr,
	&edt_ft5x06_attr_report_rate.dattr.attr,
	NULL
};

static const struct attribute_group edt_ft5x06_attr_group = {
	.attrs = edt_ft5x06_attrs,
};

#ifdef CONFIG_DEBUG_FS
static int edt_ft5x06_factory_mode(struct edt_ft5x06_ts_data *tsdata)
{
	struct i2c_client *client = tsdata->client;
	int retries = EDT_SWITCH_MODE_RETRIES;
	int ret;
	int error;

	disable_irq(client->irq);

	if (!tsdata->raw_buffer) {
		tsdata->raw_bufsize = tsdata->num_x * tsdata->num_y *
				      sizeof(u16);
		tsdata->raw_buffer = kzalloc(tsdata->raw_bufsize, GFP_KERNEL);
		if (!tsdata->raw_buffer) {
			error = -ENOMEM;
			goto err_out;
		}
	}

	/* mode register is 0x3c when in the work mode */
	if ((tsdata->version == M09) || (tsdata->version == EV) || (tsdata->version == FT5316DME))
		goto m09_out;

	error = edt_ft5x06_register_write(tsdata, WORK_REGISTER_OPMODE, 0x03);
	if (error) {
		dev_err(&client->dev,
			"failed to switch to factory mode, error %d\n", error);
		goto err_out;
	}

	tsdata->factory_mode = true;
	do {
		mdelay(EDT_SWITCH_MODE_DELAY);
		/* mode register is 0x01 when in factory mode */
		ret = edt_ft5x06_register_read(tsdata, FACTORY_REGISTER_OPMODE);
		if (ret == 0x03)
			break;
	} while (--retries > 0);

	if (retries == 0) {
		dev_err(&client->dev, "not in factory mode after %dms.\n",
			EDT_SWITCH_MODE_RETRIES * EDT_SWITCH_MODE_DELAY);
		error = -EIO;
		goto err_out;
	}

	return 0;

err_out:
	kfree(tsdata->raw_buffer);
	tsdata->raw_buffer = NULL;
	tsdata->factory_mode = false;
	enable_irq(client->irq);

	return error;

m09_out:
	dev_err(&client->dev,"No factory mode support for M09\n");
	return -EINVAL;

}

static int edt_ft5x06_work_mode(struct edt_ft5x06_ts_data *tsdata)
{
	struct i2c_client *client = tsdata->client;
	int retries = EDT_SWITCH_MODE_RETRIES;
	struct edt_reg_addr reg_addr = tsdata->reg_addr;
	int ret;
	int error;

	/* mode register is 0x01 when in the factory mode */
	error = edt_ft5x06_register_write(tsdata, FACTORY_REGISTER_OPMODE, 0x1);
	if (error) {
		dev_err(&client->dev,
			"failed to switch to work mode, error: %d\n", error);
		return error;
	}

	tsdata->factory_mode = false;

	do {
		mdelay(EDT_SWITCH_MODE_DELAY);
		/* mode register is 0x01 when in factory mode */
		ret = edt_ft5x06_register_read(tsdata, WORK_REGISTER_OPMODE);
		if (ret == 0x01)
			break;
	} while (--retries > 0);

	if (retries == 0) {
		dev_err(&client->dev, "not in work mode after %dms.\n",
			EDT_SWITCH_MODE_RETRIES * EDT_SWITCH_MODE_DELAY);
		tsdata->factory_mode = true;
		return -EIO;
	}

	if (tsdata->raw_buffer)
		kfree(tsdata->raw_buffer);
	tsdata->raw_buffer = NULL;

	/* restore parameters */
	if ( tsdata->version == FT5316DME)
    {
        edt_ft5x06_register_write(tsdata,  0, 0x40);    // Set to Test-Mode
        edt_ft5x06_register_write(tsdata,  reg_addr.reg_gain,       tsdata->gain);
        mdelay(FT5316DME_SWITCH_MODE_DELAY);
        edt_ft5x06_register_write(tsdata,  0, 0x00);    // Set to Operating-Mode
    }else
    {
    	edt_ft5x06_register_write(tsdata,  reg_addr.reg_threshold,  tsdata->threshold);
        edt_ft5x06_register_write(tsdata,  reg_addr.reg_gain,       tsdata->gain);
    	edt_ft5x06_register_write(tsdata,  reg_addr.reg_offset,     tsdata->offset);
    }
	if (reg_addr.reg_report_rate)
	  edt_ft5x06_register_write(tsdata,reg_addr.reg_report_rate,tsdata->report_rate);

	enable_irq(client->irq);

	return 0;
}

static int edt_ft5x06_debugfs_mode_get(void *data, u64 *mode)
{
	struct edt_ft5x06_ts_data *tsdata = data;

	*mode = tsdata->factory_mode;

	return 0;
};

static int edt_ft5x06_debugfs_mode_set(void *data, u64 mode)
{
	struct edt_ft5x06_ts_data *tsdata = data;
	int retval = 0;

	if (mode > 1)
		return -ERANGE;

	mutex_lock(&tsdata->mutex);

	if (mode != tsdata->factory_mode) {
		retval = mode ? edt_ft5x06_factory_mode(tsdata) :
			        edt_ft5x06_work_mode(tsdata);
	}

	mutex_unlock(&tsdata->mutex);

	return retval;
};

DEFINE_SIMPLE_ATTRIBUTE(debugfs_mode_fops, edt_ft5x06_debugfs_mode_get,
			edt_ft5x06_debugfs_mode_set, "%llu\n");

static int edt_ft5x06_debugfs_raw_data_open(struct inode *inode,
					    struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t edt_ft5x06_debugfs_raw_data_read(struct file *file,
				char __user *buf, size_t count, loff_t *off)
{
	struct edt_ft5x06_ts_data *tsdata = file->private_data;
	struct i2c_client *client = tsdata->client;
	int retries  = EDT_RAW_DATA_RETRIES;
	int val, i, error;
	size_t read = 0;
	int colbytes;
	char wrbuf[3];
	u8 *rdbuf;

	if (*off < 0 || *off >= tsdata->raw_bufsize)
		return 0;

	mutex_lock(&tsdata->mutex);

	if (!tsdata->factory_mode || !tsdata->raw_buffer) {
		error = -EIO;
		goto out;
	}

	error = edt_ft5x06_register_write(tsdata, 0x08, 0x01);
	if (error) {
		dev_dbg(&client->dev,
			"failed to write 0x08 register, error %d\n", error);
		goto out;
	}

	do {
		msleep(EDT_RAW_DATA_DELAY);
		val = edt_ft5x06_register_read(tsdata, 0x08);
		if (val < 1)
			break;
	} while (--retries > 0);

	if (val < 0) {
		error = val;
		dev_dbg(&client->dev,
			"failed to read 0x08 register, error %d\n", error);
		goto out;
	}

	if (retries == 0) {
		dev_dbg(&client->dev,
			"timed out waiting for register to settle\n");
		error = -ETIMEDOUT;
		goto out;
	}

	rdbuf = tsdata->raw_buffer;
	colbytes = tsdata->num_y * sizeof(u16);

	wrbuf[0] = 0xf5;
	wrbuf[1] = 0x0e;
	for (i = 0; i < tsdata->num_x; i++) {
		wrbuf[2] = i;  /* column index */
		error = edt_ft5x06_ts_readwrite(tsdata->client,
						sizeof(wrbuf), wrbuf,
						colbytes, rdbuf);
		if (error)
			goto out;

		rdbuf += colbytes;
	}

	read = min_t(size_t, count, tsdata->raw_bufsize - *off);
	if (copy_to_user(buf, tsdata->raw_buffer + *off, read)) {
		error = -EFAULT;
		goto out;
	}

	*off += read;
out:
	mutex_unlock(&tsdata->mutex);
	return error ?: read;
};


static const struct file_operations debugfs_raw_data_fops = {
	.open = edt_ft5x06_debugfs_raw_data_open,
	.read = edt_ft5x06_debugfs_raw_data_read,
};

static void edt_ft5x06_ts_prepare_debugfs(struct edt_ft5x06_ts_data *tsdata,
			      const char *debugfs_name)
{
	tsdata->debug_dir = debugfs_create_dir(debugfs_name, NULL);
	if (!tsdata->debug_dir)
		return;

	debugfs_create_u16("num_x", S_IRUSR, tsdata->debug_dir, &tsdata->num_x);
	debugfs_create_u16("num_y", S_IRUSR, tsdata->debug_dir, &tsdata->num_y);

	debugfs_create_file("mode", S_IRUSR | S_IWUSR,
			    tsdata->debug_dir, tsdata, &debugfs_mode_fops);
	debugfs_create_file("raw_data", S_IRUSR,
			    tsdata->debug_dir, tsdata, &debugfs_raw_data_fops);
}

static void edt_ft5x06_ts_teardown_debugfs(struct edt_ft5x06_ts_data *tsdata)
{
	if (tsdata->debug_dir)
		debugfs_remove_recursive(tsdata->debug_dir);
	kfree(tsdata->raw_buffer);
}

#else

static inline void
edt_ft5x06_ts_prepare_debugfs(struct edt_ft5x06_ts_data *tsdata,
			      const char *debugfs_name)
{
}

static inline void
edt_ft5x06_ts_teardown_debugfs(struct edt_ft5x06_ts_data *tsdata)
{
}

#endif /* CONFIG_DEBUGFS */

static int edt_ft5x06_ts_reset(struct i2c_client *client,
			struct edt_ft5x06_ts_data *tsdata)
{
	int error;

	if (gpio_is_valid(tsdata->wake_pin)) {
		error = devm_gpio_request_one(&client->dev,
					tsdata->wake_pin, GPIOF_OUT_INIT_LOW,
					"edt-ft5x06 wake");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as wake pin, error %d\n",
				tsdata->wake_pin, error);
			return error;
		}

		msleep(5);
		gpio_set_value(tsdata->wake_pin, 1);
	}
	if (gpio_is_valid(tsdata->reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		error = devm_gpio_request_one(&client->dev,
					tsdata->reset_pin, GPIOF_OUT_INIT_LOW,
					"edt-ft5x06 reset");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				tsdata->reset_pin, error);
			return error;
		}
		msleep(70);
		gpio_set_value(tsdata->reset_pin, 1);
		msleep(600);
	}

	return 0;
}

static int edt_ft5x06_ts_identify(struct i2c_client *client,
					struct edt_ft5x06_ts_data *tsdata,
					char *fw_version)
{
	u8 rdbuf[EDT_NAME_LEN];
	char *p;
	int error;
	char *model_name = tsdata->name;

	/* see what we find if we assume it is a M06 *
	 * if we get less than EDT_NAME_LEN, we don't want
	 * to have garbage in there
	 */
	memset(rdbuf, 0, sizeof(rdbuf));
	error = edt_ft5x06_ts_readwrite(client, 1, "\xbb",
					EDT_NAME_LEN - 1, rdbuf);
	if (error)
		return error;

	/* if we find something consistent, stay with that assumption
	 * at least M09 wont send 3 bytes here…
	 */
	if (!(strncasecmp(rdbuf + 1, "EP0", 3))) {
		tsdata->version = M06;

		/* remove last '$' end marker */
		rdbuf[EDT_NAME_LEN - 1] = '\0';
		if (rdbuf[EDT_NAME_LEN - 2] == '$')
			rdbuf[EDT_NAME_LEN - 2] = '\0';
			/* look for Model/Version separator */
		p = strchr(rdbuf, '*');
		if (p)
			*p++ = '\0';
		strlcpy(model_name, rdbuf + 1, EDT_NAME_LEN);
		strlcpy(fw_version, p ? p : "", EDT_NAME_LEN);
		dev_err(&client->dev, model_name);
	        dev_err(&client->dev, "\n");
	} else 
	{
		/*since there are only two versions around… (M06, M09) */
		if (tsdata->alternate_version == FT5316DME)
            tsdata->version = FT5316DME;    // AZ-Display touch enumerates as M09. Differentiate for Gain-setting
        else
            tsdata->version = M09;

		error = edt_ft5x06_ts_readwrite(client, 1, "\xA6", 2, rdbuf);

		dev_err(&client->dev, "********************  FT5x06_Name @0xA6:0x%x err:0x%x\n",rdbuf[0], error);

		if (error)
			return error;

		strlcpy(fw_version, rdbuf, 2);

		error = edt_ft5x06_ts_readwrite(client, 1, "\xA8",
				1, rdbuf);

		dev_err(&client->dev, "********************  FT5x06_Name @0xA8:0x%x 0x%x\n",rdbuf[0], error);
	        dev_err(&client->dev, " Model ");

		if (error)
			return error;

		snprintf(model_name, EDT_NAME_LEN, "EP0%i%i0M09", rdbuf[0]>>4,rdbuf[0]&0x0F);
		dev_err(&client->dev, model_name);
	        dev_err(&client->dev, "\n");
	}

	return 0;
}

#define EDT_ATTR_CHECKSET(name, reg) \
do {								\
	if (pdata->name >= edt_ft5x06_attr_##name.limit_low &&		\
	    pdata->name <= edt_ft5x06_attr_##name.limit_high)		\
		edt_ft5x06_register_write(tsdata, reg, pdata->name);	\
} while (0)

#define EDT_GET_PROP(name, reg) {				\
	u32 val;						\
	if (of_property_read_u32(np, #name, &val) == 0)		\
		edt_ft5x06_register_write(tsdata, reg, val);	\
}

static void edt_ft5x06_ts_get_dt_defaults(struct device_node *np,
					struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr *reg_addr = &tsdata->reg_addr;

    if ( tsdata->version == FT5316DME)
    {
    	u32 val;						\
	    if (of_property_read_u32(np, "gain", &val) == 0)
        {
            edt_ft5x06_register_write(tsdata,  0, 0x40);    // Set to Test-Mode
		    edt_ft5x06_register_write(tsdata, reg_addr->reg_gain, val);
            mdelay(FT5316DME_SWITCH_MODE_DELAY);
            edt_ft5x06_register_write(tsdata,  0, 0x00);    // Set to Operating-Mode
        }
    }else
    {   
        EDT_GET_PROP(threshold, reg_addr->reg_threshold);
	    EDT_GET_PROP(gain, reg_addr->reg_gain);
	    EDT_GET_PROP(offset, reg_addr->reg_offset);
    }
}

static void
edt_ft5x06_ts_get_defaults(struct edt_ft5x06_ts_data *tsdata,
			   const struct edt_ft5x06_platform_data *pdata)
{
	struct edt_reg_addr *reg_addr = &tsdata->reg_addr;

	if (!pdata->use_parameters)
		return;

	/* pick up defaults from the platform data */
	EDT_ATTR_CHECKSET(threshold, reg_addr->reg_threshold);
	EDT_ATTR_CHECKSET(gain, reg_addr->reg_gain);
	EDT_ATTR_CHECKSET(offset, reg_addr->reg_offset);
	if (reg_addr->reg_report_rate != NO_REGISTER)
		EDT_ATTR_CHECKSET(report_rate, reg_addr->reg_report_rate);
}

static void
edt_ft5x06_ts_get_parameters(struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr *reg_addr = &tsdata->reg_addr;

	tsdata->threshold = edt_ft5x06_register_read(tsdata,
						     reg_addr->reg_threshold);
	
    if ( tsdata->version == FT5316DME)
    {
        edt_ft5x06_register_write(tsdata,  0, 0x40);    // Set to Test-Mode
        tsdata->gain = edt_ft5x06_register_read(tsdata, reg_addr->reg_gain);
        mdelay(FT5316DME_SWITCH_MODE_DELAY);
        edt_ft5x06_register_write(tsdata,  0, 0x00);    // Set to Operating-Mode
    }else
    {
        tsdata->gain = edt_ft5x06_register_read(tsdata, reg_addr->reg_gain);
    }
	
    tsdata->offset = edt_ft5x06_register_read(tsdata, reg_addr->reg_offset);
	if (reg_addr->reg_report_rate != NO_REGISTER)
		tsdata->report_rate = edt_ft5x06_register_read(tsdata,
						reg_addr->reg_report_rate);
	tsdata->num_x = edt_ft5x06_register_read(tsdata, reg_addr->reg_num_x);
	tsdata->num_y = edt_ft5x06_register_read(tsdata, reg_addr->reg_num_y);
}

static void edt_ft5x06_ts_set_regs(struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr *reg_addr = &(tsdata->reg_addr);
	switch (tsdata->version){
		case M06:
			reg_addr->reg_threshold   = WORK_REGISTER_THRESHOLD;
			reg_addr->reg_report_rate = WORK_REGISTER_REPORT_RATE;
			reg_addr->reg_gain        = WORK_REGISTER_GAIN;
			reg_addr->reg_offset      = WORK_REGISTER_OFFSET;
			reg_addr->reg_num_x       = WORK_REGISTER_NUM_X;
			reg_addr->reg_num_y       = WORK_REGISTER_NUM_Y;
			break;
		case EV:
		case M09:
			reg_addr->reg_threshold   = M09_REGISTER_THRESHOLD;
			reg_addr->reg_report_rate = M09_REGISTER_REPORT_RATE;
			reg_addr->reg_gain        = M09_REGISTER_GAIN;
			reg_addr->reg_offset      = M09_REGISTER_OFFSET;
			reg_addr->reg_num_x       = M09_REGISTER_NUM_X;
			reg_addr->reg_num_y       = M09_REGISTER_NUM_Y;
			break;
        case FT5316DME:
			reg_addr->reg_threshold   = M09_REGISTER_THRESHOLD;
			reg_addr->reg_report_rate = M09_REGISTER_REPORT_RATE;
			reg_addr->reg_gain        = FT5316DME_REGISTER_GAIN;
			reg_addr->reg_offset      = M09_REGISTER_OFFSET;
			reg_addr->reg_num_x       = M09_REGISTER_NUM_X;
			reg_addr->reg_num_y       = M09_REGISTER_NUM_Y;
            break;
		default:
			/*just in case…*/
			reg_addr->reg_threshold = 0;
			reg_addr->reg_report_rate = 0;
			reg_addr->reg_gain = 0;
			reg_addr->reg_offset = 0;
			reg_addr->reg_num_x = 0;
			reg_addr->reg_num_y = 0;
			break;
	}

}

#ifdef CONFIG_OF
static int edt_ft5x06_i2c_ts_probe_dt(struct device *dev,
				struct edt_ft5x06_ts_data *tsdata)
{
	struct device_node *np = dev->of_node;
	const char *prop;
	u32 val;	
	/*
	 * irq_pin is not needed for DT setup.
	 * irq is associated via 'interrupts' property in DT
	 */
	tsdata->irq_pin   = -EINVAL;
	tsdata->reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
	tsdata->wake_pin  = of_get_named_gpio(np, "wake-gpios", 0);
	tsdata->swapxy    = 0;
	tsdata->prop_num_x= 0;
	tsdata->prop_num_y= 0;	
	tsdata->prop_xres = 0;
	tsdata->prop_yres = 0;	
	
	prop = of_get_property(np,    "swapxy", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->swapxy |= EDT_SWAPXY;	  

	prop = of_get_property(np,    "xreverse", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->swapxy |= EDT_XREVERSE;	  

	prop = of_get_property(np,    "yreverse", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->swapxy |= EDT_YREVERSE;	  

	if (!of_property_read_u32(np, "num_x",&val))	        tsdata->prop_num_x = val;
	if (!of_property_read_u32(np, "num_y",&val))	        tsdata->prop_num_y = val;

	if (!of_property_read_u32(np, "xres",&val))	        tsdata->prop_xres = val;
	if (!of_property_read_u32(np, "yres",&val))	        tsdata->prop_yres = val;

	prop = of_get_property(np,    "enable_wakeup", NULL);
	if (prop && (strcmp(prop,     "true")==0))		tsdata->enable_wakeup = 1;
	else		                                        tsdata->enable_wakeup = 0;


	printk("EDT Devtree: swapxy=0x%x num_x=%d, num_y=%d enable_wakeup=%d irq_pin=0x%x reset_pin=0x%x, wake_pin=0x%x\n",
		tsdata->swapxy, tsdata->prop_num_x, tsdata->prop_num_y, tsdata->enable_wakeup, tsdata->irq_pin, tsdata->reset_pin, tsdata->wake_pin);
	printk("EDT Devtree: %dx%d ", tsdata->prop_xres, tsdata->prop_yres);
	
	return 0;
}
#else
static inline int edt_ft5x06_i2c_ts_probe_dt(struct device *dev,
					struct edt_ft5x06_ts_data *tsdata)
{
	return -ENODEV;
}
#endif

void StopWm9715TouchDriver(void);
static const struct of_device_id edt_ft5x06_of_match[];

static int edt_ft5x06_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	const struct edt_ft5x06_platform_data *pdata =
						dev_get_platdata(&client->dev);
    const struct of_device_id *match;
	struct edt_ft5x06_ts_data *tsdata;
	struct input_dev *input;
	int error;
	char fw_version[EDT_NAME_LEN];

	dev_dbg(&client->dev, "probing for EDT FT5x06 I2C\n");

	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

    match = i2c_of_match_device(edt_ft5x06_of_match, client);
    if ( match)
    {
        if ( 0 == strcmp( match->compatible, "edt,edt-ft5316"))
        {
            tsdata->alternate_version = FT5316DME;
        }          
    }

	if (!pdata) {
		error = edt_ft5x06_i2c_ts_probe_dt(&client->dev, tsdata);
		if (error) {
			dev_err(&client->dev,
				"DT probe failed and no platform data present\n");
			return error;
		}
	} else {
		tsdata->reset_pin = pdata->reset_pin;
		tsdata->irq_pin   = pdata->irq_pin;
	        tsdata->swapxy    = pdata->swapxy;	 
		tsdata->wake_pin  = -EINVAL;
	}

	if (gpio_is_valid(tsdata->reset_pin))
	{  
	  error = edt_ft5x06_ts_reset(client, tsdata);
	  if (error)
		return error;
	}
	
	if( tsdata->swapxy & EDT_SWAPXY)   swapxy=1;	
	if( tsdata->swapxy & EDT_XREVERSE) xneg  =1;
	if( tsdata->swapxy & EDT_YREVERSE) yneg  =1;
	
#if 0
	if (gpio_is_valid(tsdata->irq_pin)) {
		error = devm_gpio_request_one(&client->dev, tsdata->irq_pin,
					GPIOF_IN, "edt-ft5x06 irq");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				tsdata->irq_pin, error);
			return error;
		}
	}
#endif

	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	mutex_init(&tsdata->mutex);
	tsdata->client = client;
	tsdata->input = input;
	tsdata->factory_mode = false;
	tsdata->driver_suspended    = 0;
	tsdata->driver_sendbuttonup = 0;
	
	error = edt_ft5x06_ts_identify(client, tsdata, fw_version);
	if (error) {
		dev_err(&client->dev, "touchscreen probe failed\n");
		return error;
	}

	edt_ft5x06_ts_set_regs(tsdata);

	if (!pdata)
		edt_ft5x06_ts_get_dt_defaults(client->dev.of_node, tsdata);
	else
		edt_ft5x06_ts_get_defaults(tsdata, pdata);

	edt_ft5x06_ts_get_parameters(tsdata);

	// if( tsdata->num_x + tsdata->num_y == 0 )
	// { 
        // FIXME: Since M09 Supported, numx,y are wrongly read
		if (!(strncmp(tsdata->name, "EP0700M06", 9)))
		{
			tsdata->num_x=28;
			tsdata->num_y=16;
		}
		if (  !strncmp(tsdata->name, "EP0510M09", 9) ||
		      !strncmp(tsdata->name, "EP0790M09", 9)
		   )
		{
		        dev_err(&client->dev, "TouchScreen %s fake EP0700M06\n",tsdata->name);		  
		        strcpy(tsdata->name,  "EP0700M06");
			strcpy(fw_version,    "unknown");			
			maxx = 800;            tsdata->num_x=0;
			maxy = 480;            tsdata->num_y=0;
		}
		dev_err(&client->dev, "TouchScreen got wrong Sensor Size Version:%d,nx@%d ny@%d.\n",
			tsdata->version,
			tsdata->reg_addr.reg_num_x, tsdata->reg_addr.reg_num_y);
	//}

	if( tsdata->prop_num_x ) tsdata->num_x=tsdata->prop_num_x;
	if( tsdata->prop_num_y ) tsdata->num_y=tsdata->prop_num_y;

	dev_err(&client->dev,
		"Model \"%s\", Rev. \"%s\", %dx%d sensors\n",
		tsdata->name, fw_version, tsdata->num_x, tsdata->num_y);

	input->name = tsdata->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(ABS_MT_POSITION_X,    input->absbit);  /* Say we are a modern Multi-Touch pad               */
	__set_bit(ABS_MT_POSITION_Y,    input->absbit);	 /* framework/native/services/inputflinger/EventHub.c */
	
	__set_bit(EV_SYN,    input->evbit);
	__set_bit(EV_KEY,    input->evbit);
	__set_bit(EV_ABS,    input->evbit);
	__set_bit(EV_REP,    input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(KEY_POWER, input->keybit);

	if( tsdata->num_x != 0)     maxx=tsdata->num_x * 64 - 1;
	if( tsdata->num_y != 0)     maxy=tsdata->num_y * 64 - 1;	    
	if( tsdata->prop_xres != 0) maxx=tsdata->prop_xres - 1;
	if( tsdata->prop_yres != 0) maxy=tsdata->prop_yres - 1;	    

	printk( KERN_ERR "EDT register Touch ABS_X %d ABS_Y %d,%dPoints\n",
		maxx,maxy,MAX_SUPPORT_POINTS);	
	    
	input_set_abs_params(input, ABS_X,             0, maxx, 0, 0);
	input_set_abs_params(input, ABS_Y,             0, maxy, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, maxx, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, maxy, 0, 0);

/*
	if (!pdata)
	  touchscreen_parse_of_params(input);
*/
	touchscreen_parse_properties(input, true, &tsdata->prop);

	error = input_mt_init_slots(input, MAX_SUPPORT_POINTS, 0);

	if (error) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		return error;
	}

	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					edt_ft5x06_ts_isr,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		return error;
	}

	error = sysfs_create_group(&client->dev.kobj, &edt_ft5x06_attr_group);
	if (error)
		return error;

	error = input_register_device(input);
	if (error)
		goto err_remove_attrs;

	edt_ft5x06_ts_prepare_debugfs(tsdata, dev_driver_string(&client->dev));
	device_init_wakeup(&client->dev, 1);
	
	dev_err(&client->dev,
		"EDT FT5x06 initialized: IRQ %d, WAKE Gpio %d, Reset Gpio %d. Probe OK\n",
		client->irq, tsdata->wake_pin, tsdata->reset_pin);

#if 0
	if( strncmp( GetFramebufferDisplayName(0), "IPAN7-EDT-18WVGA",10) == 0)
	{
	  printk( KERN_ERR "*************************** Invert PWM for IPAN7-EDT ********************\n");
	  Trizeps_Pwm_Invert(1);  
	}

#endif	
//	StopWm9715TouchDriver();
	return 0;

err_remove_attrs:
	sysfs_remove_group(&client->dev.kobj, &edt_ft5x06_attr_group);
	return error;
}

static int edt_ft5x06_ts_remove(struct i2c_client *client)
{
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);

	edt_ft5x06_ts_teardown_debugfs(tsdata);
	sysfs_remove_group(&client->dev.kobj, &edt_ft5x06_attr_group);

	return 0;
}

static int __maybe_unused edt_ft5x06_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	//	printk(KERN_ERR "*** TouchScreen edt_ft5x06_ts_suspend ***\n");
	tsdata->driver_suspended++;	
	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused edt_ft5x06_ts_resume(struct device *dev)
{
	struct i2c_client *client         = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	tsdata->driver_suspended--;	
	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(edt_ft5x06_ts_pm_ops,
			 edt_ft5x06_ts_suspend, edt_ft5x06_ts_resume);

static const struct i2c_device_id edt_ft5x06_ts_id[] = {
	{ "edt-ft5x06", 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, edt_ft5x06_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id edt_ft5x06_of_match[] = {
	{ .compatible = "edt,edt-ft5206", },
	{ .compatible = "edt,edt-ft5306", },
	{ .compatible = "edt,edt-ft5316", },
	{ .compatible = "edt,edt-ft5406", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, edt_ft5x06_of_match);
#endif

static struct i2c_driver edt_ft5x06_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "edt_ft5x06",
		.of_match_table = of_match_ptr(edt_ft5x06_of_match),
		.pm = &edt_ft5x06_ts_pm_ops,
	},
	.id_table = edt_ft5x06_ts_id,
	.probe    = edt_ft5x06_ts_probe,
	.remove   = edt_ft5x06_ts_remove,
};

module_i2c_driver(edt_ft5x06_ts_driver);

MODULE_AUTHOR("Simon Budig <simon.budig@kernelconcepts.de>");
MODULE_DESCRIPTION("EDT FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
