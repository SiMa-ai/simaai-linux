#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>

/* Algo and Data files for I2C Boot */
#include "FPGA_I2C_to_SPI_algo.h"
#include "FPGA_I2C_to_SPI_data.h"

#include "econ_fpga_fw.txt"

#define FPGA_SLAVE_ADDR 	0x0C
//#define I2C_DEBUG
s32 fpga_write_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val);
s32 fpga_write_32byte_reg(struct i2c_client *client, u16 sladdr, u16 reg, char *bytearray);
s32 fpga_read_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 * val);
short int ispProcessI2C(struct i2c_client *client);

/***************************************************************
*
* Supported I2C versions.
*
***************************************************************/

const char * const g_szSupportedVersions[] = { "_I2C1.0", 0 };
char * g_pucDataArray= g_pucDataArray_val;
char * g_pucAlgoArray= g_pucAlgoArray_val;

/*************************************************************
*                                                            *
* EXTERNAL VARIABLES                                         *
*                                                            *
*************************************************************/

extern struct gpio_desc *fpga_nprogram_gpio;
extern void toggle_gpio_fpga(struct gpio_desc *gpio, int val);

int ReadBytesAndSendNACK( struct i2c_client *client,int length, unsigned char *a_ByteRead, int NAck );
int SendBytesAndCheckACK(struct i2c_client *client,int length, unsigned char *a_bByteSend);
int ToggleTRST(int toggle);
void EnableHardware(struct i2c_client *client);
void DisableHardware(struct i2c_client *client);
extern u8 au8Buf[5];
//extern write_size=0;
//extern write_buf_i=0;
int write_size=0,write_buf_i=0;
u8 au8Buf[5];

unsigned short g_usCpu_Frequency  = 1000;   /*Enter your CPU frequency here, unit in MHz.*/
unsigned short restart;
unsigned short read;
unsigned short write;
int msg_count;
unsigned short device_write;
struct i2c_msg msg[163505];
unsigned char *buf[163505];

/*************************************************************
*                                                            *
* EXTERNAL VARIABLES                                         *
*                                                            *
*************************************************************/

int g_iDataSize;

int g_iAlgoSize;

int g_iMovingAlgoIndex = 0;
int g_iMainDataIndex = 0;
int g_iMovingDataIndex = 0;
int g_iRepeatIndex = 0;
unsigned short g_usLCOUNTSize	= 0;

unsigned short g_usDataType;
int g_iTDIIndex = 0;
int g_iTDOIndex = 0;
int g_iMASKIndex = 0;
unsigned char g_ucCompressCounter = 0;
int  g_iLoopMovingIndex = 0;
int  g_iLoopDataMovingIndex = 0;
int  g_iLoopIndex = 0;
short int ispVMSend( struct i2c_client *client,unsigned int a_uiDataSize );

int TDI_buffer_index=0;
	
/*************************************************************
*                                                            *
* GETBYTE                                                    *
*                                                            *
* INPUT:                                                     *
*     a_iCurrentIndex: the current index to access.          *
*                                                            *
*     a_cAlgo: 1 if the return byte is to be retrieved from  *
*     the algorithm array, 0 if the byte is to be retrieved  *
*     from the data array.                                   *
*                                                            *
* RETURN:                                                    *
*     This function returns a byte of data from either the   *
*     algorithm or data array.  It returns -1 if out of      *
*     bounds.                                                *
*                                                            *
*************************************************************/
unsigned char GetByte( int a_iCurrentIndex, char a_cAlgo );
short int ispVMShift( struct i2c_client *client,char a_cCommand );
unsigned int ispVMDataSize(void);
short int ispVMShiftExec( unsigned int a_uiDataSize );
short int ispVMRead( struct i2c_client *client,unsigned int a_uiDataSize );
void ispVMComment(void);

#ifndef _OPCODE_H_
#define _OPCODE_H_

//*=====================================================
//*
//* I2C Opcode Table
//*
//* Version 1.0.0
//*		


// transmission related opcode def
#define	I2C_STARTTRAN		0x10
#define	I2C_RESTARTTRAN		0x11
#define I2C_ENDTRAN			0x12
#define	I2C_TRANSOUT		0x13
#define	I2C_TRANSIN			0x14
#define	I2C_RUNCLOCK		0x15
#define I2C_WAIT		 0x16
#define I2C_LOOP		 0x17  
#define I2C_ENDLOOP		 0x18
#define I2C_TDI		     0x19
#define I2C_CONTINUE	 0x1A
#define I2C_TDO		     0x1B
#define I2C_MASK		 0x1C
#define I2C_BEGIN_REPEAT 0x1D
#define I2C_END_REPEAT	 0x1E
#define I2C_END_FRAME	 0x1F
#define I2C_DATA		 0x20
#define I2C_PROGRAM		 0x21
#define I2C_VERIFY		 0x22
#define I2C_DTDI		 0x23
#define I2C_DTDO		 0x24
#define I2C_COMMENT		 0x25
#define I2C_ENDCOMMENT	 0x26
#define I2C_TRST		 0x27
#define I2C_ENDVME		 0x7F

/*************************************************************
*                                                            *
* ERROR DEFINITIONS                                          *
*                                                            *
*************************************************************/

#define ERR_VERIFY_FAIL				-1
#define ERR_FIND_ALGO_FILE			-2
#define ERR_FIND_DATA_FILE			-3
#define ERR_WRONG_VERSION			-4
#define ERR_ALGO_FILE_ERROR			-5
#define ERR_DATA_FILE_ERROR			-6
#define ERR_OUT_OF_MEMORY			-7
#define ERR_VERIFY_ACK_FAIL			-8

/*************************************************************
*                                                            *
* DATA TYPE REGISTER BIT DEFINITIONS                         *
*                                                            *
*************************************************************/

#define SDR_DATA		0x0001	/*** Current command is SDR ***/
#define TDI_DATA		0x0002	/*** Command contains TDI ***/
#define TDO_DATA		0x0004	/*** Command contains TDO ***/
#define MASK_DATA		0x0008	/*** Command contains MASK ***/
#define DTDI_DATA		0x0010	/*** Verification flow ***/
#define DTDO_DATA		0x0020	/*** Verification flow ***/
#define COMPRESS		0x0040	/*** Compressed data file ***/
#define COMPRESS_FRAME	0x0080	/*** Compressed data frame ***/

#endif

int fpga_write_i2c(struct i2c_client *client, u16 sladdr,  u8 * val, u32 count)
{
	int ret;

	struct i2c_msg msg = {
		.addr = sladdr,
		.flags = 0,
		.len = count,
		.buf = val,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register ret = %d!\n",
				ret);
		return ret;
	}
	return 0;
}

s32 fpga_write_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val)
{
	u8 bcount = 3;
	u8 au8Buf[3] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (fpga_write_i2c(client, sladdr,au8Buf, bcount) < 0) {
		dev_err(&client->dev,
				"%s:write reg error: reg = 0x%x,val = 0x%x\n", __func__,
				reg, val);
		return -EIO;
	}

	return 0;
}

int fpga_read_i2c(struct i2c_client *client, u16 sladdr, u8 * val, u32 count)
{
	int ret;
	struct i2c_msg msg = {
		.addr = sladdr,
		.flags = 0,
		.buf = val,
	};

	msg.flags = I2C_M_RD;
	msg.len = count;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	return 0;

 err:
	dev_err(&client->dev, "Failed reading register ret = %d!\n", ret);
	return ret;
}

s32 fpga_read_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 * val)
{
	u8 bcount;
	u8 au8RegBuf[2] = { 0 };
	u8 au8RdVal[1] = { 0 };

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;
	bcount = 2;

	if (fpga_write_i2c(client, sladdr, au8RegBuf, bcount) < 0) {
		dev_err(&client->dev,"%s:write reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	bcount = 1;
	if (fpga_read_i2c(client,sladdr, au8RdVal, bcount) < 0) {
		dev_err(&client->dev,"%s:read reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	*val = au8RdVal[0];

	return 0;
}

s32 fpga_read_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u16 * val)
{
	u8 bcount;
	u8 au8RegBuf[2] = { 0 };
	u8 au8RdVal[2] = { 0 };

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;
	bcount = 2;

	if (fpga_write_i2c(client, sladdr, au8RegBuf, bcount) < 0) {
		dev_err(&client->dev,"%s:write reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	bcount = 2;
	if (fpga_read_i2c(client,sladdr, au8RdVal, bcount) < 0) {
		dev_err(&client->dev,"%s:read reg error:reg=0x%x\n", __func__, reg);
		return -EIO;
	}

	*val = (au8RdVal[0] << 8 | au8RdVal[1]);

	return 0;
}

s32 fpga_write_16b_reg(struct i2c_client *client, u16 sladdr, u16 reg, u16 val)
{
	u8 bcount = 4;
	u8 au8Buf[4] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >> 8;
	au8Buf[3] = val & 0xff;

	if (fpga_write_i2c(client, sladdr,au8Buf, bcount) < 0) {
		dev_err(&client->dev,
			"%s:write reg error: reg = 0x%x,val = 0x%x\n", __func__,
			reg, val);
		return -EIO;
	}

	return 0;
}

s32 fpga_write_32byte_reg(struct i2c_client *client, u16 sladdr, u16 reg, char *bytearray)
{
	u32 bcount = 34;
	u8 au8Buf[34] = {0};
	int i, j = 0;

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;

	for(i=2,j=0;i<34;i++,j++)
	{
		au8Buf[i] = bytearray[j] & 0xff;
		//pr_info("*********au8Buf[%d] = 0x%x********\n",i, au8Buf[i]);
		 
	}
	if (fpga_write_i2c(client, sladdr,au8Buf, bcount) != 0) {
		pr_err("%s:read reg error:reg=0x%x\n", __func__, reg);
		return -1;
	}
	return 0;
}

/***********Function Definitions & Declarations*****************************/
unsigned char GetByte( int a_iCurrentIndex, char a_cAlgo )
{
	unsigned char ucData = 0;
	
	if ( a_cAlgo ) { 
		/*************************************************************
		*                                                            *
		* If the current index is still within range, then return    *
		* the next byte.  If it is out of range, then return -1.     *
		*                                                            *
		*************************************************************/
		if ( a_iCurrentIndex >= g_iAlgoSize ) {
			return ( unsigned char ) 0xFF;
		}
		ucData = g_pucAlgoArray[ a_iCurrentIndex ];
	}
	else {
		/*************************************************************
		*                                                            *
		* If the current index is still within range, then return    *
		* the next byte.  If it is out of range, then return -1.     *
		*                                                            *
		*************************************************************/
		if ( a_iCurrentIndex >= g_iDataSize ) {
			return ( unsigned char ) 0xFF;
		}
	
		ucData = g_pucDataArray[ a_iCurrentIndex ];
	}

	return ucData;
}

/*************************************************************
*                                                            *
* SetI2CStartCondition                                       *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     int                                                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to issue a start sequence on the *
*     I2C Bus								                 *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

int SetI2CStartCondition(void)
{

	//SCL SDA high
	//signalSDA = 1;
	//SetI2CDelay(1);
	//signalSCL = 1;
	//SetI2CDelay(1);
	//SCL high SDA low
	//signalSDA = 0;
	//SetI2CDelay(1);
	//SCL low SDA low
	//signalSCL = 0;
	//SetI2CDelay(1);
	
	return 0;
}
/*************************************************************
*                                                            *
* SetI2CReStartCondition                                     *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None                                                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to issue a start sequence on the *
*     I2C Bus								                 *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int SetI2CReStartCondition(struct i2c_client *client)
{
restart = 1;
	return 0;
	
}
/*************************************************************
*                                                            *
* SetI2CStopCondition                                        *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None                                                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to issue a stop sequence on the  *
*     I2C Bus								                 *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int SetI2CStopCondition(struct i2c_client *client)
{
	int ret,i;
	
	if(	device_write ){
		ret = i2c_transfer(client->adapter, msg, msg_count);
		if (ret < 0) {
			dev_err(&client->dev, "Failed writing register ret = %d!\n",
					ret);
			return ret;
		}
#ifdef I2C_DEBUG
		printk("%s msg_count = %d\n",__func__,msg_count);
#endif
		for(i=0;i<msg_count;i++){
			if(msg[i].buf)
				kfree(msg[i].buf);
		}
		msg_count = 0; 
	}
	return 0;
}

/*************************************************************
*                                                            *
* READBYTESANDSENDNACK                                         *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     Returns the bit read back from the device.             *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to read the TDO pin from the     *
*     input port.                                            *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int ReadBytesAndSendNACK(struct i2c_client *client, int count, unsigned char *val , int NAck)
{
	int ret,i;
	if(restart || (msg_count == 0)){
		restart = 0;

		msg[msg_count].addr = 0x40;
		if(write){
			msg[msg_count].flags = 0;
			write = 0;
		}
		else if(read){
			msg[msg_count].flags = I2C_M_RD;
			read = 0;
		}else
			msg[msg_count].flags = 0;

		msg[msg_count].len = count/8;
		msg[msg_count].buf = val;

#ifdef I2C_DEBUG
		printk("%s msg_count = %d count = %d flags = %d bus = %d\n",__func__,msg_count,msg[msg_count].len,msg[msg_count].flags,client->adapter->nr);
#endif
		msg_count++;
		ret = i2c_transfer(client->adapter, msg, msg_count);
		if (ret < 0) {
			dev_err(&client->dev, "Failed writing register ret = %d!\n",
					ret);
			return ret;
		}
#ifdef I2C_DEBUG
		printk("%s msg_count = %d\n",__func__,msg_count);
#endif
		for(i=0;i<msg_count-1;i++){
			if(msg[i].buf)
				kfree(msg[i].buf);
		}
		msg_count = 0;
		device_write = 0;
	}
	return 0;
}
/*************************************************************
*                                                            *
* SENDBYTESANDCHECKACK                                        *
*                                                            *
* INPUT:                                                     *
*                                                            *
*     a_bByteSend: the value to determine of the pin above   *
*     will be written out or not.                            *
*                                                            *
* RETURN:                                                    *
*     true or false.                                         *
*                                                            *
* DESCRIPTION:                                               *
*     To apply the specified value to the pins indicated.    *
*     This routine will likely be modified for specific      *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
#if 0
static s32 cam_write_32byte_data(struct i2c_client *client, char *bytearray,int length)
{
	int i=0,j=0,count=0,ret;
	u32 bcount = length;
	u8 au8Buf[length];// = {0};

	//    au8Buf[2] = val >> 8;
	for(i=0,j=0;i<length;i++,j++)
	{
		au8Buf[i] = bytearray[j] & 0xff; 
	}
	if (cam_write(client, au8Buf, bcount) != 0) {
		while(count<100)
		{
			ret=cam_write(client, au8Buf, bcount);		
			count=count+1;
			if(ret==0)
			{
				break;
			}
		}
		pr_err("WRITE ERROR %s:read reg error:\n", __func__);
		return -1;
	}


	return 0;
}
#endif


int SendBytesAndCheckACK(struct i2c_client *client, int count, unsigned char *val)
{
int i;
	if((count%8 == 0) && (count/8 == 1)){
	 	if(val[0] == 0x80)
		{
			write = 1;
			return 0;
		}
		else if(val[0] == 0x81)
		{
			read = 1;
			return 0;
		}
	//	return 0;
	}
	if(restart || (msg_count == 0)){
		restart = 0;
		buf[msg_count] = kzalloc(2048, GFP_KERNEL);
		if (!buf[msg_count]){
			printk("Memory alocation failed\n");
			return -ENOMEM;
		}
		for(i=0;i<(count/8);i++)
			{
			buf[msg_count][i] = val[i];
}


		msg[msg_count].addr = 0x40;
		if(write){
			msg[msg_count].flags = 0;
			write = 0;
		}
		else if(read){
			msg[msg_count].flags = I2C_M_RD;
			read = 0;
		}else
			msg[msg_count].flags = 0;

		msg[msg_count].len = count/8;
		msg[msg_count].buf = buf[msg_count];
#ifdef I2C_DEBUG
		printk("%s msg_count = %d count = %d flags = %d bus = %d\n",__func__,msg_count,msg[msg_count].len,msg[msg_count].flags,client->adapter->nr);
#endif
		msg_count++;
		device_write = 1;
	}else{
#ifdef I2C_DEBUG
		printk("++++++++\n");
#endif
		for(i=0;i<count/8;i++)
		{
		msg[msg_count-1].buf[msg[msg_count-1].len+i] = val[i];
		}

		msg[msg_count-1].addr = 0x40;
		if(write){
			msg[msg_count-1].flags = 0;
			write = 0;
		}
		else if(read){
			msg[msg_count-1].flags = I2C_M_RD;
			read = 0;
		}else
			msg[msg_count-1].flags = 0;

		msg[msg_count-1].len += count/8;
#ifdef I2C_DEBUG
		printk("%s msg_count = %d count = %d flags = %d bus = %d\n",__func__,msg_count,msg[msg_count-1].len,msg[msg_count-1].flags,client->adapter->nr);
#endif

		device_write = 1;
	}
	return 0;
}

/*************************************************************
*                                                            *
* SetI2CDelay                                                *
*                                                            *
* INPUT:                                                     *
*     a_uiDelay: number of waiting time.                     *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     Users must devise their own timing procedures to       *
*     ensure the specified minimum delay is observed when    *
*     using different platform.  The timing function used    *
*     here is for PC only by hocking the clock chip.         *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
void SetI2CDelay( unsigned int a_msTimeDelay )
{	
	unsigned short loop_index     = 0;
	unsigned short ms_index       = 0;
	unsigned short us_index       = 0;
//mdelay(a_msTimeDelay);
#if 1
	/*Users can replace the following section of code by their own*/
	for( ms_index = 0; ms_index < a_msTimeDelay; ms_index++)
	{
		/*Loop 1000 times to produce the milliseconds delay*/
		for (us_index = 0; us_index < 1000; us_index++)
		{ /*each loop should delay for 1 microsecond or more.*/
			loop_index = 0;
			do {
				/*The NOP fakes the optimizer out so that it doesn't toss out the loop code entirely*/
				//__asm NOP
			}while (loop_index++ < ((g_usCpu_Frequency/8)+(+ ((g_usCpu_Frequency % 8) ? 1 : 0))));/*use do loop to force at least one loop*/
		}
	}
#endif
}
/*************************************************************
*                                                            *
* ENABLEHARDWARE                                             *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function is called to enable the hardware.        *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

void EnableHardware(struct i2c_client *client)
{
//	SetI2CStartCondition();
//	SetI2CStopCondition(client);
}

/*************************************************************
*                                                            *
* DISABLEHARDWARE                                            *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function is called to disable the hardware.       *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

void DisableHardware(struct i2c_client *client)
{
	//SetI2CStopCondition(client);
}

/*************************************************************
*                                                            *
* ISPVMDATASIZE                                              *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     This function returns a number indicating the size of  *
*     the instruction.                                       *
*                                                            *
* DESCRIPTION:                                               *
*     This function returns a number.  
*************************************************************/
unsigned int ispVMDataSize(void)
{
	unsigned int uiSize = 0;
	unsigned char ucCurrentByte = 0;
	unsigned char ucIndex = 0;
	
	while ( ( ucCurrentByte = GetByte( g_iMovingAlgoIndex++, 1 ) ) & 0x80 ) {
		
		uiSize |= ( ( unsigned int ) ( ucCurrentByte & 0x7F ) ) << ucIndex;
		ucIndex += 7;
	}
	uiSize |= ( ( unsigned int ) ( ucCurrentByte & 0x7F ) ) << ucIndex;
	
	return uiSize;
}

/*************************************************************
*                                                            *
* ISPVMREAD                                                  *
*                                                            *
* INPUT:                                                     *
*     a_uiDataSize: this argument is the size of the         *
*     command.                                               *
*                                                            *
* RETURN:                                                    *
*     The return value is 0 if passing, and -1 if failing.   *
*                                                            *
* DESCRIPTION:                                               *
*     This function reads a data stream from the device and  *
*     compares it to the expected TDO.                       *
*                                                            *
*************************************************************/
short int ispVMRead( struct i2c_client *client,unsigned int a_uiDataSize )
{
	unsigned int uiIndex = 0;
	unsigned int usBufferIndex = 0;
	unsigned short usErrorCount = 0;
	unsigned char ucTDOByte = 0;
	unsigned char ucTDIByte = 0;
	unsigned char ucMaskByte = 0;
	unsigned char ucCurBit = 0;
	unsigned char cInDataByte = 0;
	unsigned char *InData = NULL;
	unsigned char *ReadData = NULL;
	unsigned char *TmpReadData = NULL;
	if((InData = (unsigned char *) kmalloc((a_uiDataSize+7)/8+1,GFP_KERNEL)) == NULL)
		return ERR_OUT_OF_MEMORY;
	if((ReadData = (unsigned char *) kmalloc((a_uiDataSize+7)/8+1,GFP_KERNEL)) == NULL)
		return ERR_OUT_OF_MEMORY;
	for ( uiIndex = 0; uiIndex < a_uiDataSize; uiIndex++ ) { 
		if ( uiIndex % 8 == 0 ) {
			if ( g_usDataType & TDI_DATA ) {
				
				/*************************************************************
				*                                                            *
				* If the TDI_DATA flag is set, then grab the next byte from  *
				* the algo array and increment the TDI index.                *
				*                                                            *
				*************************************************************/
				ucTDIByte = GetByte( g_iTDIIndex++, 1 );
				InData[usBufferIndex++] = ucTDIByte;
				
			}
			else if ( g_usDataType & DTDI_DATA ){
				
				/*************************************************************
				*                                                            *
				* If TDI_DATA is not set, then DTDI_DATA must be set.  If    *
				* the compression counter exists, then the next TDI byte     *
				* must be 0xFF.  If it doesn't exist, then get next byte     *
				* from data file array.                                      *
				*                                                            *
				*************************************************************/
				if ( g_ucCompressCounter ) {
					g_ucCompressCounter--;
					ucTDIByte = ( unsigned char ) 0xFF;
					InData[usBufferIndex++] = ucTDIByte;
				}
				else {
					ucTDIByte = GetByte( g_iMovingDataIndex++, 0 );
					InData[usBufferIndex++] = ucTDIByte;

					/*************************************************************
					*                                                            *
					* If the frame is compressed and the byte is 0xFF, then the  *
					* next couple bytes must be read to determine how many       *
					* repetitions of 0xFF are there.  That value will be stored  *
					* in the variable g_ucCompressCounter.                       *
					*                                                            *
					*************************************************************/
					if ( ( g_usDataType & COMPRESS_FRAME ) && ( ucTDIByte == ( unsigned char ) 0xFF ) ) {
						g_ucCompressCounter = GetByte( g_iMovingDataIndex++, 0 );
						g_ucCompressCounter--;
					}
				}
			}
			else
			{
				
				InData[usBufferIndex++] = 0xFF;
			}
		}
	}

	if(InData)
		kfree(InData);
	InData = NULL;	
	if ( g_usDataType & DTDI_DATA ){
		if ( GetByte( g_iMovingDataIndex++, 0 ) != I2C_END_FRAME ) {
			if (ReadData)
				kfree(ReadData);
			ReadData = NULL;
			return(ERR_DATA_FILE_ERROR);
		}
		if ( g_usDataType & COMPRESS ) {
			if ( g_usDataType & DTDO_DATA ) {
				g_usDataType &= ~( COMPRESS_FRAME );
				if ( GetByte( g_iMovingDataIndex++, 0 ) ) {
					g_usDataType |= COMPRESS_FRAME;
				}
			}
		}
	}
	usBufferIndex = 0;
	if( a_uiDataSize == 32)
	{
		
		
		if(ReadBytesAndSendNACK(client,a_uiDataSize, ReadData, 1))
		{
			if (ReadData)
				kfree(ReadData);
			ReadData = NULL;
			return(-1);
		}
	}
	else
	{
		if(ReadBytesAndSendNACK(client,a_uiDataSize, ReadData, 0))
		{
			if (ReadData)
				kfree(ReadData);
			ReadData = NULL;
			return(-1);
		}
	}


#ifdef I2C_DEBUG
		printk("\nEXPECTED TDO (" );
#endif
	
	usBufferIndex = 0;
	for ( uiIndex = 0; uiIndex < a_uiDataSize; uiIndex++ ) { 
		if ( uiIndex % 8 == 0 ) {
			if ( g_usDataType & TDO_DATA ) {
				/*************************************************************
				*                                                            *
				* If the TDO_DATA flag is set, then grab the next byte from  *
				* the algo array and increment the TDO index.                *
				*                                                            *
				*************************************************************/
				ucTDOByte = GetByte( g_iTDOIndex++, 1 );
				
#ifdef I2C_DEBUG
				printk("%.2X g_iTDOIndex=%d", ucTDOByte,g_iTDOIndex );
#endif
			}
			else {
				/*************************************************************
				*                                                            *
				* If TDO_DATA is not set, then DTDO_DATA must be set.  If    *
				* the compression counter exists, then the next TDO byte     *
				* must be 0xFF.  If it doesn't exist, then get next byte     *
				* from data file array.                                      *
				*                                                            *
				*************************************************************/
				if ( g_ucCompressCounter ) {
					g_ucCompressCounter--;
					ucTDOByte = ( unsigned char ) 0xFF;
				}
				else {
					ucTDOByte = GetByte( g_iMovingDataIndex++, 0 );
#ifdef I2C_DEBUG
					printk( "%.2X g_iMovingDataIndex=%d", ucTDOByte,g_iMovingDataIndex );
#endif

					/*************************************************************
					*                                                            *
					* If the frame is compressed and the byte is 0xFF, then the  *
					* next couple bytes must be read to determine how many       *
					* repetitions of 0xFF are there.  That value will be stored  *
					* in the variable g_ucCompressCounter.                       *
					*                                                            *
					*************************************************************/
					if ( ( g_usDataType & COMPRESS_FRAME ) && ( ucTDOByte == ( unsigned char ) 0xFF ) ) {
						g_ucCompressCounter = GetByte( g_iMovingDataIndex++, 0 );
						g_ucCompressCounter--;
					}
				}
			}

			if ( g_usDataType & MASK_DATA ) {
				ucMaskByte = GetByte( g_iMASKIndex++, 1 );
			}
			else { 
				ucMaskByte = ( unsigned char ) 0xFF;
			}
			cInDataByte = ReadData[usBufferIndex++];
		}

		ucCurBit = (unsigned char)(((cInDataByte << uiIndex%8) & 0x80) ? 0x01 : 0x00);	
		if ( ( ( ( ucMaskByte << uiIndex % 8 ) & 0x80 ) ? 0x01 : 0x00 ) ) {	
			if ( ucCurBit != ( unsigned char ) ( ( ( ucTDOByte << uiIndex % 8 ) & 0x80 ) ? 0x01 : 0x00 ) ) {
				usErrorCount++;  
			}
		}
	}
#ifdef I2C_DEBUG
		printk(  ");\n" );
#endif
#ifdef I2C_DEBUG
		printk("ACTUAL TDO (" );
int usDataSizeIndex;
		for ( usDataSizeIndex = 0; usDataSizeIndex < (unsigned short)( ( a_uiDataSize + 7 ) / 8 ) ; usDataSizeIndex++ ) {
			cInDataByte = ReadData[ usDataSizeIndex ];
			printk( "%.2X usDataSizeIndex=%d", cInDataByte,usDataSizeIndex );
			if ( usDataSizeIndex % 40 == 39 ) {
				printk( "\n\t\t" );
			}
		}
		printk( ");\n" );		
#endif //I2C_DEBUG

	if (ReadData)
		kfree(ReadData);
	ReadData = NULL;
	if ( usErrorCount > 0 ) {
		printk("DEBUG ispVMRead return -1\n");
		return -1;
	}
	return 0;
}



/*************************************************************
*                                                            *
* ISPVMSHIFT                                                 *
*                                                            *
* INPUT:                                                     *
*     a_cCommand: this argument specifies either the SIR or  *
*     SDR command.                                           *
*                                                            *
* RETURN:                                                    *
*     The return value indicates whether the SIR/SDR was     *
*     processed successfully or not.  A return value equal   *
*     to or greater than 0 is passing, and less than 0 is    *
*     failing.                                               *
*                                                            *
* DESCRIPTION:                                               *
*     This function is the entry point to execute an SIR or  *
*     SDR command to the device.                             *
*                                                            *
*************************************************************/
short int ispVMShift( struct i2c_client *client,char a_cCommand )
{
	short int siRetCode = 0;
	unsigned int uiDataSize = ispVMDataSize();
	int flag_RW,flag_W,err,ret;
#ifdef I2C_DEBUG
	printk( "SDR %d ", uiDataSize );
#endif /* I2C_DEBUG */
	char *buf;
	buf = devm_kzalloc(&client->dev,40,GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}	
	/*************************************************************
	 *                                                            *
	 * Clear any existing SDR instructions from the data type *
	 * register.                                                  *
	 *                                                            *
	*************************************************************/
	
	g_usDataType &= ~( SDR_DATA );

	/*************************************************************
	*                                                            *
	* Set the data type register to indicate that it's executing *
	* an SDR instruction.  Move state machine to DRPAUSE,        *
	* SHIFTDR.  If header data register exists, then issue       *
	* bypass.                                                    *
	*                                                            *
	*************************************************************/
	g_usDataType |= SDR_DATA;		
		
	/*************************************************************
	*                                                            *
	* Set the appropriate index locations.  If error then return *
	* error code immediately.                                    *
	*                                                            *
	*************************************************************/

	siRetCode = ispVMShiftExec( uiDataSize );
	
	if ( siRetCode < 0 ) {
		return siRetCode;
	}
	/*************************************************************
	*                                                            *
	* Execute the command to the device.  If TDO exists, then    *
	* read from the device and verify.  Else only TDI exists     *
	* which must send data to the device only.                   *
	*                                                            *
	*************************************************************/

	if ( ( g_usDataType & TDO_DATA ) || ( g_usDataType & DTDO_DATA ) ) {

		siRetCode = ispVMRead( client,uiDataSize );
		/*************************************************************
		*                                                            *
		* A frame of data has just been read and verified.  If the   *
		* DTDO_DATA flag is set, then check to make sure the next    *
		* byte in the data array, which is the last byte of the      *
		* frame, is the END_FRAME byte.                              *
		*                                                            *
		*************************************************************/
		if ( g_usDataType & DTDO_DATA ) {
			if ( GetByte( g_iMovingDataIndex++, 0 ) != I2C_END_FRAME ) {
				siRetCode = ERR_DATA_FILE_ERROR;
			}
		}
	}
	else {
		siRetCode = ispVMSend( client,uiDataSize );
		/*************************************************************
		*                                                            *
		* A frame of data has just been sent.  If the DTDI_DATA flag *
		* is set, then check to make sure the next byte in the data  *
		* array, which is the last byte of the frame, is the         *
		* END_FRAME byte.                                            *
		*                                                            *
		*************************************************************/
		if ( g_usDataType & DTDI_DATA ) {
			if ( GetByte( g_iMovingDataIndex++, 0 ) != I2C_END_FRAME ) {
				siRetCode = ERR_DATA_FILE_ERROR;
			}
		}
	}	
	return siRetCode;
}

/*************************************************************
*                                                            *
* ISPVMSHIFTEXEC                                             *
*                                                            *
* INPUT:                                                     *
*     a_uiDataSize: this holds the size of the command.      *
*                                                            *
* RETURN:                                                    *
*     Returns 0 if passing, -1 if failing.                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function handles the data in the commands         *
*     by either decompressing the data or setting the        *
*     respective indexes to point to the appropriate         *
*     location in the algo or data array.                    *
*                                                            *
*************************************************************/
short int ispVMShiftExec( unsigned int a_uiDataSize )
{
	unsigned char ucDataByte = 0;
	//unsigned char TDI_DataByte = 0;	
	/*************************************************************
	*                                                            *
	* Reset the data type register.                              *
	*                                                            *
	*************************************************************/

	g_usDataType &= ~( TDI_DATA + TDO_DATA + MASK_DATA + DTDI_DATA + DTDO_DATA + COMPRESS_FRAME );

	
	/*************************************************************
	*                                                            *
	* Convert the size from bits to byte.                        *
	*                                                            *
	*************************************************************/

	if ( a_uiDataSize % 8 ) {
		a_uiDataSize = a_uiDataSize / 8 + 1;
	}
	else {
		a_uiDataSize = a_uiDataSize / 8;
	}
	
	/*************************************************************
	*                                                            *
	* Begin extracting the command.                              *
	*                                                            *
	*************************************************************/
	//TDI_Buffer_Data_size=a_uiDataSize;
	//Read_Buffer_Data_size=a_uiDataSize;
	while ( ( ucDataByte = GetByte( g_iMovingAlgoIndex++, 1 ) ) != I2C_CONTINUE ) {
		 
		switch ( ucDataByte ) {
		
		case I2C_TDI:
			/*************************************************************
			*                                                            *
			* Set data type register to indicate TDI data and set TDI    *
			* index to the current algorithm location.                   *
			*                                                            *
			*************************************************************/
		
			g_usDataType |= TDI_DATA;
			g_iTDIIndex = g_iMovingAlgoIndex;
			g_iMovingAlgoIndex += a_uiDataSize;
			TDI_buffer_index = g_iTDIIndex;
		
			break;
		case I2C_DTDI:
			/*************************************************************
			*                                                            *
			* Set data type register to indicate DTDI data and check the *
			* next byte to make sure it's the DATA byte.  DTDI indicates *
			* that the data should be read from the data array, not the  *
			* algo array.                                                *
			*                                                            *
			*************************************************************/
			
			g_usDataType |= DTDI_DATA;
			if ( GetByte( g_iMovingAlgoIndex++, 1 ) != I2C_DATA ) {
				
				return ERR_ALGO_FILE_ERROR;
			}

			/*************************************************************
			*                                                            *
			* If the COMPRESS flag is set, read the next byte from the   *
			* data file array.  If the byte is true, then that indicates *
			* the frame was compressable.  Note that even though the     *
			* overall data file was compressed, certain frames may not   *
			* be compressable that is why this byte must be checked.     *
			*                                                            *
			*************************************************************/
		
			if ( g_usDataType & COMPRESS ) {
		
				if ( GetByte( g_iMovingDataIndex++, 0 ) ) {
		
					g_usDataType |= COMPRESS_FRAME;
					
				}
			}
			break;
		case I2C_TDO:
			/*************************************************************
			*                                                            *
			* Set data type register to indicate TDO data and set TDO    *
			* index to the current algorithm location.                   *
			*                                                            *
			
			*************************************************************/
		
			g_usDataType |= TDO_DATA;
			g_iTDOIndex = g_iMovingAlgoIndex;
			g_iMovingAlgoIndex += a_uiDataSize;	
			break;
		case I2C_DTDO:
			/*************************************************************
			*                                                            *
			* Set data type register to indicate DTDO data and check the *
			* next byte to make sure it's the DATA byte.  DTDO indicates *
			* that the data should be read from the data array, not the  *
			* algo array.                                                *
			*                                                            *
			*************************************************************/
			
			g_usDataType |= DTDO_DATA;
			if ( GetByte( g_iMovingAlgoIndex++, 1 ) != I2C_DATA ) {
				return ERR_ALGO_FILE_ERROR;
			}

			/*************************************************************
			*                                                            *
			* If the COMPRESS flag is set, read the next byte from the   *
			* data file array.  If the byte is true, then that indicates *
			* the frame was compressable.  Note that even though the     *
			* overall data file was compressed, certain frames may not   *
			* be compressable that is why this byte must be checked.     *
			*                                                            *
			*************************************************************/
			
			if ( g_usDataType & COMPRESS ) {
				if ( !(g_usDataType & DTDI_DATA) ) {
					if ( GetByte( g_iMovingDataIndex++, 0 ) ) {
						g_usDataType |= COMPRESS_FRAME;
					}
				}
			}
			break;
		case I2C_MASK:
			/*************************************************************
			*                                                            *
			* Set data type register to indicate MASK data.  Set MASK    *
			* location index to current algorithm array position.        *
			*                                                            *
			
			*************************************************************/
			
			g_usDataType |= MASK_DATA;
			g_iMASKIndex = g_iMovingAlgoIndex;
			g_iMovingAlgoIndex += a_uiDataSize;
			break;
		default:
			
			
			/*************************************************************
			*                                                            *
			* Unrecognized or misplaced opcode.  Return error.           *
			*                                                            *
			*************************************************************/
			return ERR_ALGO_FILE_ERROR;
		}
	}  
	
	/*************************************************************
	*                                                            *
	* Reached the end of the instruction.  Return passing.       *
	*                                                            *
	*************************************************************/

			
	return 0;
}

void ispVMComment()
{
	unsigned char currentByte  = 0;	
	do{
		currentByte = GetByte( g_iMovingAlgoIndex++, 1 );
		if(currentByte == I2C_ENDCOMMENT)
			break;
		msleep(1);
		//printk( "%c", currentByte);
	}while(currentByte != I2C_ENDCOMMENT);	
	//printk( "\n" );
}

/***************************************************************
*
* ispVMLoop
*
* Perform the function call upon by the REPEAT opcode.
* Memory is to be allocated to store the entire loop from REPEAT to ENDLOOP.
* After the loop is stored then execution begin. The REPEATLOOP flag is set
* on the g_usFlowControl register to indicate the repeat loop is in session
* and therefore fetch opcode from the memory instead of from the file.
*
***************************************************************/

short int ispVMLoop(struct i2c_client *client,unsigned short a_usLoopCount)
{
	short int siRetCode = 0;
	unsigned short usContinue = 1;
	unsigned char ucOpcode = 0;
	unsigned int uiDataSize = 0;
	int intDelay = 0;
	
	/*************************************************************
	*                                                            *
	* Set the repeat index to the first byte in the repeat loop. *
	*                                                            *
	*************************************************************/

	g_iLoopMovingIndex = g_iMovingAlgoIndex;
	g_iLoopDataMovingIndex = g_iMovingDataIndex;

	for ( g_iLoopIndex = 0 ; g_iLoopIndex < g_usLCOUNTSize; g_iLoopIndex++ ) 
	{
		usContinue	= 1;
		/*************************************************************
		*                                                            *
		* Initialize the current algorithm index to the beginning of *
		* the repeat index before each repeat loop.                  *
		*                                                            *
		*************************************************************/

		g_iMovingAlgoIndex = g_iLoopMovingIndex;
		g_iMovingDataIndex = g_iLoopDataMovingIndex;

		while ( usContinue ) 
		{
			ucOpcode = GetByte( g_iMovingAlgoIndex++, 1 );
			switch ( ucOpcode ) 
			{
				case I2C_STARTTRAN:
#ifdef I2C_DEBUG
			printk("Start Condition\n");
#endif /* I2C_DEBUG */
					siRetCode = SetI2CStartCondition();
					break;
				case I2C_RESTARTTRAN:
#ifdef I2C_DEBUG
			printk("ReStart Condition\n");
#endif /* I2C_DEBUG */
					siRetCode = SetI2CReStartCondition(client);
					break;
				case I2C_ENDTRAN:
#ifdef I2C_DEBUG
			printk("ReStart Condition\n");
#endif /* I2C_DEBUG */
					siRetCode = SetI2CStopCondition(client);
					break;
				case I2C_TRANSOUT:
					/*************************************************************
					*                                                            *
					* Execute SIR/SDR command.                                   *
					*                                                            *
					*************************************************************/
					siRetCode = ispVMShift( client,ucOpcode );
					break;
				case I2C_TRANSIN:
					/*************************************************************
					*                                                            *
					* Execute SIR/SDR command.                                   *
					*                                                            *
					*************************************************************/
					siRetCode = ispVMShift( client,ucOpcode );
					if ( siRetCode >= 0 ) {
						/****************************************************************************
						*
						* Break if intelligent programming is successful.
						*
						*****************************************************************************/

						return ( siRetCode );
					}
					else
						usContinue = 0;
					break;		
				case I2C_WAIT:
					/*************************************************************
					*                                                            *
					* Issue delay in specified time.                             *
					*                                                            *
					*************************************************************/
					intDelay = ispVMDataSize();
#ifdef I2C_DEBUG
			printk("Delay %d\n", intDelay);
#endif /* I2C_DEBUG */
					SetI2CDelay( intDelay );
					break;
				case I2C_COMMENT:
					ispVMComment();
					break;
			}
		}	
	}
	return ( siRetCode );
}




/*************************************************************
*                                                            *
* ISPVMSEND                                                  *
*                                                            *
* INPUT:                                                     *
*     a_uiDataSize: this argument is the size of the         *
*     command.                                               *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function sends a data stream to the device.       *
*                                                            *
*************************************************************/

short int ispVMSend( struct i2c_client *client, unsigned int a_uiDataSize )
{
	unsigned int iIndex;
	unsigned int usBufferIndex = 0;
	unsigned char ucCurByte = 0;
	unsigned char *g_pucInData = NULL;
	if((g_pucInData = (unsigned char *) kmalloc((a_uiDataSize+7)/8+1,GFP_KERNEL)) == NULL)
		return -1;



	/*************************************************************
	*                                                            *
	* Begin processing the data to the device.                   *
	*                                                            *
	*************************************************************/

	for ( iIndex = 0; iIndex < a_uiDataSize; iIndex++ ) { 
		if ( iIndex % 8 == 0 ) { 
			if ( g_usDataType & TDI_DATA ) {
				/*************************************************************
				*                                                            *
				* If the TDI_DATA flag is set, then grab the next byte from  *
				* the algo array and increment the TDI index.                *
				*                                                            *
				*************************************************************/
			
				ucCurByte = GetByte( g_iTDIIndex++, 1 );
			}
			else {
				
				/*************************************************************
				*                                                            *
				* If TDI_DATA flag is not set, then DTDI_DATA flag must have *
				* already been set.  If the compression counter exists, then *
				* the next TDI byte must be 0xFF.  If it doesn't exist, then *
				* get next byte from data file array.                        *
				*                                                            *
				*************************************************************/
				if ( g_ucCompressCounter ) {
					g_ucCompressCounter--;
					ucCurByte = ( unsigned char ) 0xFF;
				}
				else {
					ucCurByte = GetByte( g_iMovingDataIndex++, 0 );
					
					/*************************************************************
					*                                                            *
					* If the frame is compressed and the byte is 0xFF, then the  *
					* next couple bytes must be read to determine how many       *
					* repetitions of 0xFF are there.  That value will be stored  *
					* in the variable g_ucCompressCounter.                       *
					*                                                            *
					*************************************************************/

					if ( ( g_usDataType & COMPRESS_FRAME ) && ( ucCurByte == ( unsigned char ) 0xFF ) ) {
						g_ucCompressCounter = GetByte( g_iMovingDataIndex++, 0 );
						g_ucCompressCounter--;
					}
				}
			}
			g_pucInData[usBufferIndex++] = ucCurByte;
		}
	}	
	
	if(SendBytesAndCheckACK(client,a_uiDataSize, g_pucInData))
	{
		printk("Failed to get ACK when send byte.\n");
		if(g_pucInData)
			kfree(g_pucInData);
		g_pucInData = NULL;
		return ERR_VERIFY_ACK_FAIL;
	}
#ifdef I2C_DEBUG
		printk("TDI (" );
		int cInDataByte = 0;
		int cDataByte = 0;
int usDataSizeIndex;
		for ( usDataSizeIndex = (unsigned short)( ( a_uiDataSize + 7 ) / 8 ); usDataSizeIndex > 0 ; usDataSizeIndex-- ) {
			cInDataByte = g_pucInData[ usDataSizeIndex - 1 ];
			cDataByte = 0x00;

			/****************************************************************************
			*
			* Flip cMaskByte and store it in cDataByte.
			*
			*****************************************************************************/

			for ( usBufferIndex = 0; usBufferIndex < 8; usBufferIndex++ ) {
				cDataByte <<= 1;
				if ( cInDataByte & 0x01 ) {
					cDataByte |= 0x01;
				}
				cInDataByte >>= 1;
			}
			printk( "%.2X", cDataByte );
			if ( ( ( ( a_uiDataSize + 7 ) / 8 ) - usDataSizeIndex ) % 40 == 39 ) {
				printk( "\n\t\t" );
			}
		}
		printk( ");\n" );		
#endif //I2C_DEBUG
	if(g_pucInData)
		kfree(g_pucInData);
	g_pucInData = NULL;
	return 0;
}

extern short int ispProcessI2C(struct i2c_client *client)
{
	unsigned char ucOpcode  = 0;
	unsigned char ucState   = 0;
	unsigned char ucTRST    = 0;
	short int siRetCode     = 0;
	static char cProgram    = 0;
	unsigned int uiDataSize = 0;
	int iLoopCount          = 0;
	unsigned int iMovingAlgoIndex = 0;
	int intDelay            = 0;
	
	/*************************************************************
	*                                                            *
	* Begin processing the I2C algorithm and data files.         *
	*                                                            *
	*************************************************************/
	
	while ( ( ucOpcode = GetByte( g_iMovingAlgoIndex++, 1 ) ) != 0xFF ) 
	{
		/*************************************************************
		*                                                            *
		* This switch statement is the main switch that represents   *
		* the core of the embedded processor.                        *
		*                                                            *
		*************************************************************/
		switch ( ucOpcode ) 
		{
		case I2C_STARTTRAN:
#ifdef I2C_DEBUG
			printk("Start Condition\n");
#endif /* I2C_DEBUG */
			siRetCode = SetI2CStartCondition();
			break;
		case I2C_RESTARTTRAN:
#ifdef I2C_DEBUG
			printk("ReStart Condition\n");
#endif /* I2C_DEBUG */
			siRetCode = SetI2CReStartCondition(client);
			break;
		case I2C_ENDTRAN:
#ifdef I2C_DEBUG
			printk("Stop Condition\n");
#endif /* I2C_DEBUG */
			siRetCode = SetI2CStopCondition(client);
			break;
		case I2C_TRANSOUT:
		case I2C_TRANSIN:
			/*************************************************************
			*                                                            *
			* Execute SIR/SDR command.                                   *
			*                                                            *
			*************************************************************/
			siRetCode = ispVMShift( client,ucOpcode );
			break;		
		case I2C_WAIT:
			/*************************************************************
			*                                                            *
			* Issue delay in specified time.                             *
			*                                                            *
			*************************************************************/
			intDelay = ispVMDataSize();
#ifdef I2C_DEBUG
			printk("Delay %d\n", intDelay);
#endif /* I2C_DEBUG */
			SetI2CDelay( intDelay );
			break;
		case I2C_BEGIN_REPEAT:
			/*************************************************************
			*                                                            *
			* Execute repeat loop.                                       *
			*                                                            *
			*************************************************************/

			uiDataSize = ispVMDataSize();

			switch ( GetByte( g_iMovingAlgoIndex++, 1 ) ) {
			case I2C_PROGRAM:
				/*************************************************************
				*                                                            *
				* Set the main data index to the moving data index.  This    *
				* allows the processor to remember the beginning of the      *
				* data.  Set the cProgram variable to true to indicate to    *
				* the verify flow later that a programming flow has been     *
				* completed so the moving data index must return to the      *
				* main data index.                                           *
				*                                                            *
				*************************************************************/
				g_iMainDataIndex = g_iMovingDataIndex;
				cProgram = 1;
				break;
			case I2C_VERIFY:
				/*************************************************************
				*                                                            *
				* If the static variable cProgram has been set, then return  *
				* the moving data index to the main data index because this  *
				* is a erase, program, verify operation.  If the programming *
				* flag is not set, then this is a verify only operation thus *
				* no need to return the moving data index.                   *
				*                                                            *
				*************************************************************/
				if ( cProgram ) {
					g_iMovingDataIndex = g_iMainDataIndex;
					cProgram = 0;
				}
				break;
			}

			/*************************************************************
			*                                                            *
			* Set the repeat index to the first byte in the repeat loop. *
			*                                                            *
			*************************************************************/

			g_iRepeatIndex = g_iMovingAlgoIndex;

			for ( ; uiDataSize > 0; uiDataSize-- ) {
				/*************************************************************
				*                                                            *
				* Initialize the current algorithm index to the beginning of *
				* the repeat index before each repeat loop.                  *
				*                                                            *
				*************************************************************/

				g_iMovingAlgoIndex = g_iRepeatIndex;

				/*************************************************************
				*                                                            *
				* Make recursive call.                                       *
				*                                                            *
				*************************************************************/

				siRetCode = ispProcessI2C(client);
				if ( siRetCode < 0 ) {
					break;
				}
			}
			break;
		case I2C_END_REPEAT:
			/*************************************************************
			*                                                            *
			* Exit the current repeat frame.                             *
			*                                                            *
			*************************************************************/
			return siRetCode;
			break;
		case I2C_LOOP:
			/*************************************************************
			*                                                            *
			* Execute repeat loop.                                       *
			*                                                            *
			*************************************************************/

			g_usLCOUNTSize = (short int)ispVMDataSize();			

#ifdef I2C_DEBUG
			printk( "LoopCount %d\n", g_usLCOUNTSize );
#endif
			siRetCode = ispVMLoop( client,( unsigned short ) g_usLCOUNTSize );
			if ( siRetCode != 0 ) {
				return ( siRetCode );
			}			
			break;
		case I2C_ENDLOOP:
			/*************************************************************
			*                                                            *
			* Exit the current repeat frame.                             *
			*                                                            *
			*************************************************************/			
			
			break;
		case I2C_COMMENT:
			
			ispVMComment();
			break;
		case I2C_TRST:
			
			ucTRST = GetByte( g_iMovingAlgoIndex++, 1 );
#ifdef I2C_DEBUG
			printk( "CRESET %d\n", ucTRST ); //nprogram pin
#endif
			toggle_gpio_fpga(fpga_nprogram_gpio, ucTRST);
			break;
		case I2C_ENDVME:
			/*************************************************************
			*                                                            *
			* If the ENDVME token is found and g_iMovingAlgoIndex is     *
			* greater than or equal to g_iAlgoSize, then that indicates  *
			* the end of the chain.  If g_iMovingAlgoIndex is less than  *
			* g_iAlgoSize, then that indicates that there are still more *
			* devices to be processed.                                   *
			*                                                            *
			*************************************************************/
			
			if ( g_iMovingAlgoIndex >= g_iAlgoSize ) {
				return siRetCode;
			}
			break;
		default:
			/*************************************************************
			*                                                            *
			* Unrecognized opcode.  Return with file error.              *
			*                                                            *
			*************************************************************/
			
			
			return ERR_ALGO_FILE_ERROR;
		}
		
		if ( siRetCode < 0 ) {
			return siRetCode;
		}
	}	
	return ERR_ALGO_FILE_ERROR;
}

int write_spi_data_to_fpga(struct i2c_client *client)
{
	int retcode = 0, c = 0;
	uint16_t data = 0;

	uint8_t fpga_data = 0;

	char *array_traverse_0 = g_pucAlgoArray_spi_0;
	int array_traverse_length = g_pucAlgoArray_spi_0_size;

	int last_byte_length = array_traverse_length / 32;
	last_byte_length *= 32;

	retcode = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0004, array_traverse_length & 0xff);
	if (retcode)
	{
		dev_err(&client->dev, "Unable to disable I2C Bypass %s(%d)\n",__func__,__LINE__);
		return retcode;
	}

	retcode = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0005, (array_traverse_length >> 8) & 0xff);
	if (retcode)
	{
		dev_err(&client->dev, "Unable to disable I2C Bypass %s(%d)\n",__func__,__LINE__);
		return retcode;
	}

	retcode = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0006, (array_traverse_length >> 16) & 0xff);
	if (retcode)
	{
		dev_err(&client->dev, "Unable to disable I2C Bypass %s(%d)\n",__func__,__LINE__);
		return retcode;
	}

	dev_info(&client->dev, "Waiting for SPI erase to complete...\n");
	do {
		retcode = fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0003, &fpga_data);
		if (retcode)
		{
			dev_err(&client->dev, "Unable to read data from FPGA %s(%d)",__func__, __LINE__);
			return retcode;
		}
		mdelay(5);
	}	while ( !(fpga_data & 0x01) );
	dev_info(&client->dev, "SPI Erase is completed, Writing SPI data...\n");
	dev_info(&client->dev, "Waiting for spi write to complete...\n");

	while(c<array_traverse_length)
	{
	
		retcode = fpga_write_32byte_reg(client, FPGA_SLAVE_ADDR, 0x0007, array_traverse_0);
		if(retcode != 0) {
			dev_err(&client->dev," %s(%d) i2c error while writing register address - %d\n",
					__func__,__LINE__, retcode);
			return retcode;
		}
		array_traverse_0=array_traverse_0+32;
		if(c==last_byte_length)
		{
			retcode = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0003, 0x06);
			if (retcode)
			{
				dev_err(&client->dev, "Unable to disable I2C Bypass %s(%d)\n",__func__,__LINE__);
				return retcode;
			}
		}
		else
		{
			retcode = fpga_write_reg(client, FPGA_SLAVE_ADDR, 0x0003, 0x04);
			if (retcode)
			{
				dev_err(&client->dev, "Unable to disable I2C Bypass %s(%d)\n",__func__,__LINE__);
				return retcode;
			}
		}
		c=c+32;
		do {
			retcode = fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0003, &fpga_data);
			if (retcode)
			{
				dev_err(&client->dev, "Unable to read data from FPGA %s(%d)",__func__, __LINE__);
				return retcode;
			}
			mdelay(5);
		}	while ( (fpga_data & 0x04) );
	}

	//Check if the spi flash write is complete
	do {
		retcode = fpga_read_reg(client, FPGA_SLAVE_ADDR, 0x0003, &fpga_data);
		if (retcode)
		{
			dev_err(&client->dev, "Unable to read data from FPGA %s(%d)",__func__, __LINE__);
			return retcode;
		}
		mdelay(5);
	}	while ( !(fpga_data & 0x08) );

	printk("SPI Flash Write is completed successfully.\n");

	return 0;
}

short int fpga_init(struct i2c_client *client)
{

	char szFileVersion[ 9 ] = { 0 };
	short int siRetCode     = 0;
	int iIndex              = 0;
	int cVersionIndex       = 0;
	/*************************************************************
	*                                                            *
	* VARIABLES INITIALIZATION                                   *
	*                                                            *
	*************************************************************/

	g_usDataType       = 0;
	g_iMovingAlgoIndex = 0;
	g_iMovingDataIndex = 0;

	/*************************************************************
	*                                                            *
	* Open the algorithm file, get the size in bytes, allocate   *
	* memory, and read it in.                                    *
	*                                                            *
	*************************************************************/

	if ( GetByte( g_iMovingDataIndex++, 0 ) ) {
		
			g_usDataType |= COMPRESS;
			
		}

	/***************************************************************
	*
	* Read and store the version of the VME file.
	*
	***************************************************************/

	for ( iIndex = 0; iIndex < strlen(g_szSupportedVersions[0]); iIndex++ ) {
		szFileVersion[ iIndex ] = GetByte( g_iMovingAlgoIndex++, 1 );
	 
	}



	/***************************************************************
	*
	* Compare the VME file version against the supported version.
	*
	***************************************************************/

	for ( cVersionIndex = 0; g_szSupportedVersions[ cVersionIndex ] != 0; cVersionIndex++ ) {
		for ( iIndex = 0; iIndex < strlen(g_szSupportedVersions[cVersionIndex]); iIndex++ ) {
			
			if ( szFileVersion[ iIndex ] != g_szSupportedVersions[ cVersionIndex ][ iIndex ] ) {
				siRetCode = ERR_WRONG_VERSION;
				break;
			}	
			siRetCode = 0;
		}

		if ( siRetCode == 0 ) {

			/***************************************************************
			*
			* Found matching version, break.
			*
			***************************************************************/

			break;
		}
	}


	if ( siRetCode < 0 ) {

		/***************************************************************
		*
		* VME file version failed to match the supported versions.
		*
		***************************************************************/

	//	free( g_pucAlgoArray );
	//	if ( g_pucDataArray ) {
	//		free( g_pucDataArray );
	//	}
		g_pucAlgoArray = NULL;
		g_pucDataArray = NULL;
		return ERR_WRONG_VERSION;
	}
	/*************************************************************
	*                                                            *
	* Start the hardware.                                        *
	*                                                            *
	*************************************************************/


    	//EnableHardware(client);
	
	/*************************************************************
	*                                                            *
	* Begin processing algorithm and data file.                  *
	*                                                            *
	*************************************************************/


	siRetCode = ispProcessI2C(client);


	/*************************************************************
	*                                                            *
	* Stop the hardware.                                         *
	*                                                            *
	*************************************************************/

 	//DisableHardware(client);

	/*************************************************************
	*                                                            *
	* Free dynamic memory and return value.                      *
	*                                                            *
	*************************************************************/

/*	kfree( g_pucAlgoArray );
	if ( g_pucDataArray ) {
		kfree( g_pucDataArray );
	}
	g_pucAlgoArray = NULL;
	g_pucDataArray = NULL;
*/


	return ( siRetCode );
}
