/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Based on v4l2 econ FPGA driver
 *
 * Copyright (c) 2026 e-con systems
 */

#define FPGA_SLAVE_ADDR	0x0D
//#define I2C_DEBUG
//s32 fpga_write_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 val);
static s32 fpga_write_32byte_reg(struct i2c_client *client, u16 sladdr, u16 reg, char *bytearray);
//s32 fpga_read_reg(struct i2c_client *client, u16 sladdr, u16 reg, u8 *val);
//short int ispProcessI2C(struct i2c_client *client);

/***************************************************************
 *
 * Supported I2C versions.
 *
 ***************************************************************/

const char * const g_szSupportedVersions[] = { "_I2C1.0", 0 };
char *g_pucDataArray = g_pucDataArray_val;
char *g_pucAlgoArray = g_pucAlgoArray_val;

/*************************************************************
 *                                                            *
 * EXTERNAL VARIABLES                                         *
 *                                                            *
 *************************************************************/

extern struct gpio_desc *fpga_nprogram_gpio;
extern void toggle_gpio_fpga(struct gpio_desc *gpio, int value);

static int ReadBytesAndSendNACK(struct i2c_client *client, int length,
		unsigned char *a_ByteRead, int NAck);
static int SendBytesAndCheckACK(struct i2c_client *client, int length,
		unsigned char *a_bByteSend);
//static int ToggleTRST(int toggle);
static void EnableHardware(struct i2c_client *client);
static void DisableHardware(struct i2c_client *client);
//extern u8 au8Buf[5];
//extern write_size=0;
//extern write_buf_i=0;
int write_size = 0, write_buf_i = 0;
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

int g_iMovingAlgoIndex;
int g_iMainDataIndex;
int g_iMovingDataIndex;
int g_iRepeatIndex;
unsigned short g_usLCOUNTSize;

unsigned short g_usDataType;
int g_iTDIIndex;
int g_iTDOIndex;
int g_iMASKIndex;
unsigned char g_ucCompressCounter;
int  g_iLoopMovingIndex;
int  g_iLoopDataMovingIndex;
int  g_iLoopIndex;
static short int ispVMSend(struct i2c_client *client, unsigned int a_uiDataSize);

int TDI_buffer_index;

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
static unsigned char GetByte(int a_iCurrentIndex, char a_cAlgo);
static short int ispVMShift(struct i2c_client *client, char a_cCommand);
static unsigned int ispVMDataSize(void);
static short int ispVMShiftExec(unsigned int a_uiDataSize);
static short int ispVMRead(struct i2c_client *client, unsigned int a_uiDataSize);
static void ispVMComment(void);

#ifndef _OPCODE_H_
#define _OPCODE_H_

/* =====================================================
 * I2C Opcode Table
 * Version 1.0.0
 */

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
