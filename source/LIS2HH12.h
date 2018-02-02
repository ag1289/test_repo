/*
 * LIS2HH12.h
 *
 *  Created on: 9 nov. 2017
 *      Author: agarcia
 */

#ifndef LIS2HH12_H_
#define LIS2HH12_H_

/* LIS2HH12 I2C address according SA0 pin level*/

#define LIS2HH_SA0_H_ADDR	0x1D
#define LIS2HH_SA0_L_ADDR	0x1E

#define LIS2HH_ID			0x41
/* LIS2HH12 register map */

#define LIS2HH_TEMP_L			0x0B
#define LIS2HH_TEMP_H  		    0x0C
#define LIS2HH_ACT_THS  		0x1E
#define LIS2HH_ACT_DUR  		0x1F
#define LIS2HH_CTRL1			0x20
#define LIS2HH_CTRL2			0x21
#define LIS2HH_CTRL3			0x22
#define LIS2HH_CTRL4			0x23
#define LIS2HH_CTRL5			0x24
#define LIS2HH_CTRL6			0x25
#define LIS2HH_CTRL7			0x26
#define LIS2HH_STATUS  		    0x27
#define LIS2HH_OUT_X_L			0x28
#define LIS2HH_OUT_X_H			0x29
#define LIS2HH_OUT_Y_L			0x2A
#define LIS2HH_OUT_Y_H			0x2B
#define LIS2HH_OUT_Z_L			0x2C
#define LIS2HH_OUT_Z_H			0x2D
#define LIS2HH_FIFO_CTRL  		0x2E
#define LIS2HH_FIFO_SRC 		0x2F
#define LIS2HH_IG_CFG1 		    0x30
#define LIS2HH_IG_SRC1 		    0x31
#define LIS2HH_IG_THS_X1	    0x32
#define LIS2HH_IG_THS_Y1	    0x33
#define LIS2HH_IG_THS_Z1	    0x34
#define LIS2HH_IG_DUR1 		    0x35
#define LIS2HH_IG_CFG2 		    0x36
#define LIS2HH_IG_SRC2 		    0x37
#define LIS2HH_IG_THS2 		    0x38
#define LIS2HH_IG_DUR2 		    0x39
#define LIS2HH_XL_REFERENCE	    0x3A
#define LIS2HH_XH_REFERENCE	    0x3B
#define LIS2HH_YL_REFERENCE	    0x3C
#define LIS2HH_YH_REFERENCE	    0x3D
#define LIS2HH_ZL_REFERENCE	    0x3E
#define LIS2HH_ZH_REFERENCE	    0x3F
#define LIS2HH_WHO_AM_I			0x0F  // device identification register

//STATUS REGISTER bit mask
#define LIS2HH_STATUS_REG_ZYXOR 1<<7    // 1	:	new data set has over written the previous one  0	:	no overrun has occurred (default)
#define LIS2HH_STATUS_REG_ZOR   1<<6    // 0	:	no overrun has occurred (default) 1	:	new Z-axis data has over written the previous one
#define LIS2HH_STATUS_REG_YOR   1<<5    // 0	:	no overrun has occurred (default) 1	:	new Y-axis data has over written the previous one
#define LIS2HH_STATUS_REG_XOR   1<<4    // 0	:	no overrun has occurred (default) 1	:	new X-axis data has over written the previous one
#define LIS2HH_STATUS_REG_ZYXDA 1<<3    // 0	:	a new set of data is not yet avvious one  1	:	a new set of data is available
#define LIS2HH_STATUS_REG_ZDA   1<<2    // 0	:	a new data for the Z-Axis is not availvious one 1	:	a new data for the Z-Axis is available
#define LIS2HH_STATUS_REG_YDA   1<<1    // 0	:	a new data for the Y-Axis is not available 1	:	a new data for the Y-Axis is available
#define LIS2HH_STATUS_REG_XDA   1<<0    // 0	:	a new data for the X-Axis is not available
#define LIS2HH_DATAREADY_BIT    LIS3DH_STATUS_REG_ZYXDA

#define MEMS_SUCCESS	0
#define MEMS_ERROR		1

// CONTROL REGISTER 1 BITS MASK
#define LIS2HH_HR				1<<7
#define LIS2HH_ODR2			    1<<6
#define LIS2HH_ODR1			    1<<5
#define LIS2HH_ODR0			    1<<4
#define LIS2HH_BDU				1<<3
#define LIS2HH_ZEN				1<<2
#define LIS2HH_YEN				1<<1
#define LIS2HH_XEN				1<<0

//CONTROL REGISTER 2 BITS MASK
#define LIS2HH_DFC1				1<<6
#define LIS2HH_DFC0				1<<5
#define LIS2HH_HPM1     		1<<4
#define LIS2HH_HPM0     		1<<3
#define LIS2HH_FDS				1<<2
#define LIS2HH_HPIS2			1<<1
#define LIS2HH_HPIS1			1<<0

//CONTROL REGISTER 3 BITS MASK

#define LIS2HH_I1_FIFO_EN		1<<7
#define LIS2HH_I1_STOP_FTH		1<<6
#define LIS2HH_I1_INACT			1<<5
#define LIS2HH_I1_IG2			1<<4
#define LIS2HH_I1_IG1			1<<3
#define LIS2HH_I1_OVR			1<<2
#define LIS2HH_I1_FTH			1<<1
#define LIS2HH_I1_DRDY1			1<<0

//CONTROL REGISTER 4 BITS MASK

#define LIS2HH_BW2				1<<7
#define LIS2HH_BW1				1<<6
#define LIS2HH_FS1				1<<5
#define LIS2HH_FS0				1<<4
#define LIS2HH_BW_SCALE_ODR		1<<3
#define LIS2HH_ADD_INC			1<<2
#define LIS2HH_I2C_DIS      	1<<1
#define LIS2HH_SIM				1<<0

//CONTROL REGISTER 5 BITS MASK

#define LIS2HH_DEBUG			1<<7
#define LIS2HH_SOFT_RESET		1<<6
#define LIS2HH_DEC1			    1<<5
#define LIS2HH_DEC0			    1<<4
#define LIS2HH_ST2              1<<3
#define LIS2HH_ST1              1<<2
#define LIS2HH_H_LActive        1<<1
#define LIS2HH_PP_OD            1<<0

//CONTROL REGISTER 6 BITS MASK

#define LIS2HH_BOOT			    1<<7
#define LIS2HH_I2_BOOT			1<<5
#define LIS2HH_I2_IG2			1<<4
#define LIS2HH_I2_IG1			1<<3
#define LIS2HH_I2_EMPTY			1<<2
#define LIS2HH_I2_FTH        	1<<1
#define LIS2HH_I2_RDY 			1<<0

//CONTROL REGISTER 7 BITS MASK

#define LIS2HH_DCRM2		    1<<5
#define LIS2HH_DCRM1		    1<<4
#define LIS2HH_LIR2		     	1<<3
#define LIS2HH_LIR1			    1<<2
#define LIS2HH_4D_IG2      		1<<1
#define LIS2HH_4D_IG1		    1<<0


#define LIS2HH_PITCH	0
#define LIS2HH_ROLL		1
#define LIS2HH_YAW		2

#define LIS2HH_K_R2G	180.0 / M_PI


/**
 * Accelerometer LIS2HH Full Scales in register setting.
 */
typedef enum {
	STM_LIS2HH_RANGE_2G = 0x04, /*corresponding value in register 4 setting*/
	STM_LIS2HH_RANGE_4G = 0x24, STM_LIS2HH_RANGE_8G = 0x34
} stm_lis2hh_range;

/**
 * Accelerometer LIS2HH sensitivity for each range, converted to ug/digit.
 */
typedef enum {
	STM_LIS2HH_SSTVT_2G = 61,
	STM_LIS2HH_SSTVT_4G = 122,
	STM_LIS2HH_SSTVT_8G = 244
} stm_lis2hh_sstvt;

/**
 * Accelerometer LIS2HH output data rate in register 1 setting
 */
typedef enum {
	STM_LIS2HH_ODR_OFF = 0x00,
	STM_LIS2HH_ODR10 = 0x10, /* 10Hz output data rate */
	STM_LIS2HH_ODR50 = 0x20, /* 50Hz output data rate */
	STM_LIS2HH_ODR100 = 0x30, /* 100Hz output data rate */
	STM_LIS2HH_ODR200 = 0x40, /* 200Hz output data rate */
	STM_LIS2HH_ODR400 = 0x50, /* 400Hz output data rate */
	STM_LIS2HH_ODR800 = 0x60 /* 800Hz output data rate */
} stm_lis2hh_odr;

typedef struct {
	int32_t X;
	int32_t Y;
	int32_t Z;
} AxesRaw_t;

AxesRaw_t *GetReferenceVector(void);
int32_t LIS2HH_GetStatusReg(uint8_t*);
int32_t LIS2HH_GetStatusBit(uint8_t, uint8_t*);
int32_t LIS2HH_GetAccAxesRaw(AxesRaw_t *);
float* ComputeTiltAngles(AxesRaw_t* );
float Calc2VectorAngle(AxesRaw_t * , AxesRaw_t *);
void VectorialProduct( AxesRaw_t * , AxesRaw_t * , AxesRaw_t * );
void CalculateNormalVector(AxesRaw_t * , AxesRaw_t *);
float ComputePlaneAngle(AxesRaw_t *);
int32_t LIS2HH_Init(void);
int32_t CaptureRefVector(void);

#endif /* LIS2HH12_H_ */
