#ifndef MCP794X_DRIVER_H
#define MCP794X_DRIVER_H

#include <stdint.h>
#include <time.h>

#define USER_SRAM_SIZE		64
#define USER_E2PROM_SIZE	128

// **************************************************************************************
//                   GLOBAL CONSTANTS RTCC - REGISTERS ADDRESSES 
// **************************************************************************************
#define ADDR_EEPROM 		0x57       	//  DEVICE ADDR for EEPROM   
#define ADDR_SRAM   		0x6f       	//  DEVICE ADDR for RTCC MCHP   
#define UNLOCK_CODE1		0x55		//  Unlock code 1
#define UNLOCK_CODE2		0xaa		//  Unlock code 1
#define E2ROM_BYTES 		0x80		//  E2PROM capacity in bytes 
#define SRAM_BYTES			0x40		//  SRAM capacity in bytes
#define ORG_IDCODE64        0xf0        //  Memory address of ID 64
#define ORG_IDCODE48		0xf2		//  Memory address of ID 48
#define RTC_ERROR			0x00		//  operation error byte
#define RTC_OK				0x01		//  operation error byte
#define E2ROM_PAGE_BYTES	0x08		// Number of page bytes to write

// **************************************************************************************
#define  SRAM_PTR          0x20       	//  pointer of the SRAM area (RTCC) 
#define  ADDR_EEPROM_SR    0xff       	//  STATUS REGISTER in the  EEPROM
// **************************************************************************************
#define  ADDR_SEC          0x00       	//  address of SECONDS      register 
#define  ADDR_MIN          0x01       	//  address of MINUTES      register 
#define  ADDR_HOUR         0x02       	//  address of HOURS        register 
#define  ADDR_DAY          0x03       	//  address of DAY OF WK    register 
#define  ADDR_STAT         0x03       	//  address of STATUS       register 
#define  ADDR_DATE         0x04       	//  address of DATE         register  
#define  ADDR_MNTH         0x05       	//  address of MONTH        register 
#define  ADDR_YEAR         0x06       	//  address of YEAR         register 
#define  ADDR_CTRL         0x07       	//  address of CONTROL      register 
#define  ADDR_CAL          0x08       	//  address of CALIB        register 
#define  ADDR_ULID         0x09       	//  address of UNLOCK ID    register
// **************************************************************************************
#define  ADDR_ALM0SEC      0x0a       	//  address of ALARM0 SEC   register 
#define  ADDR_ALM0MIN      0x0b       	//  address of ALARM0 MIN   register 
#define  ADDR_ALM0HR       0x0c       	//  address of ALARM0 HOUR  register 
#define  ADDR_ALM0CTL      0x0d     	//  address of ALARM0 CONTR register
#define  ADDR_ALM0DAT      0x0e   	    //  address of ALARM0 DATE  register 
#define  ADDR_ALM0MTH      0x0f       	//  address of ALARM0 MONTH register 
// **************************************************************************************
#define  ADDR_ALM1SEC      0x11        	//  address of ALARM1 SEC   register 
#define  ADDR_ALM1MIN      0x12        	//  address of ALARM1 MIN   register 
#define  ADDR_ALM1HR       0x13        	//  address of ALARM1 HOUR  register 
#define  ADDR_ALM1CTL      0x14        	//  address of ALARM1 CONTR register
#define  ADDR_ALM1DAT      0x15        	//  address of ALARM1 DATE  register 
#define  ADDR_ALM1MTH      0x16        	//  address of ALARM1 MONTH register 
// **************************************************************************************
#define  ADDR_SAVtoBAT_MIN 0x18        	//  address of T_SAVER MIN(VDD->BAT)
#define  ADDR_SAVtoBAT_HR  0x19        	//  address of T_SAVER HR (VDD->BAT) 
#define  ADDR_SAVtoBAT_DAT 0x1a        	//  address of T_SAVER DAT(VDD->BAT) 
#define  ADDR_SAVtoBAT_MTH 0x1b        	//  address of T_SAVER MTH(VDD->BAT) 
// **************************************************************************************
#define  ADDR_SAVtoVDD_MIN 0x1c        	//  address of T_SAVER MIN(BAT->VDD)
#define  ADDR_SAVtoVDD_HR  0x1d        	//  address of T_SAVER HR (BAT->VDD) 
#define  ADDR_SAVtoVDD_DAT 0x1e        	//  address of T_SAVER DAT(BAT->VDD) 
#define  ADDR_SAVtoVDD_MTH 0x1f        	//  address of T_SAVER MTH(BAT->VDD)        
// **************************************************************************************
//                  GLOBAL CONSTANTS RTCC - INITIALIZATION 
// **************************************************************************************
#define  PMbit             0x20       	//  post-meridian bit (ADDR_HOUR) 
#define  OUT_PIN           0x80       	//  = b7 (ADDR_CTRL) 
#define  SQWE              0x40       	//  SQWE = b6 (ADDR_CTRL) 
#define  ALM_NO            0x00       	//  no alarm activated        (ADDR_CTRL) 
#define  ALM_0             0x10       	//  ALARM0 is       activated (ADDR_CTRL)
#define  ALM_1             0x20       	//  ALARM1 is       activated (ADDR_CTRL)
#define  ALM_01            0x30       	//  both alarms are activated (ADDR_CTRL)
#define  MFP_01H           0x00       	//  MFP = SQVAW(01 HERZ)      (ADDR_CTRL)  
#define  MFP_04K           0x01       	//  MFP = SQVAW(04 KHZ)       (ADDR_CTRL)  
#define  MFP_08K           0x02       	//  MFP = SQVAW(08 KHZ)       (ADDR_CTRL)  
#define  MFP_32K           0x03       	//  MFP = SQVAW(32 KHZ)       (ADDR_CTRL)  
#define  MFP_64H           0x04       	//  MFP = SQVAW(64 HERZ)      (ADDR_CTRL)
#define  ALMx_POL          0x80       	//  polarity of MFP on alarm  (ADDR_ALMxCTL)   
#define  ALMxC_SEC         0x00      	//  ALARM compare on SEC      (ADDR_ALMxCTL)              
#define  ALMxC_MIN         0x10       	//  ALARM compare on MIN      (ADDR_ALMxCTL)   
#define  ALMxC_HR          0x20       	//  ALARM compare on HOUR     (ADDR_ALMxCTL)   
#define  ALMxC_DAY         0x30       	//  ALARM compare on DAY      (ADDR_ALMxCTL)   
#define  ALMxC_DAT         0x40       	//  ALARM compare on DATE     (ADDR_ALMxCTL)   
#define  ALMxC_ALL         0x70       	//  ALARM compare on all param(ADDR_ALMxCTL)    
#define  ALMx_IF           0x08       	//  MASK of the ALARM_IF      (ADDR_ALMxCTL)
#define  OSCON             0x20       	//  state of the oscillator(running or not)
#define  VBATEN            0x08       	//  enable battery for back-up  
#define  START_32KHZ       0x80       	//  start crystal: ST = b7 (ADDR_SEC)
#define  LP                0x20       	//  mask for the leap year bit(MONTH REG)   
#define  HOUR_12           0x40       	//  12 hours format   (ADDR_HOUR)
#define	 PWR_FAIL		   0x10			// Power fail flag	

#define CUT_YEAR		   1900			//  00:00:00 on January 1, 1900, Coordinated Universal Time  ISO/IEC 9899:1999
#define RTC_CENTURY	       2000			// Base RTC century		
// **************************************************************************************
//                  STRUCT DEFINITIONS 
// **************************************************************************************

typedef union 
{
	struct  
	{
		uint8_t seconds:4;
		uint8_t secondsx10:3;
		uint8_t ST:1;
	}Bits;

	uint8_t val;
}RTCReg0;

typedef union
{
	struct
	{
		uint8_t minutes:4;
		uint8_t minutesx10:3;
		uint8_t:1;
	}Bits;
	
	uint8_t val;
}RTCReg1;

typedef union
{
	struct
	{
		uint8_t hour:4;
		uint8_t hourx10:3;
		uint8_t F24H:1;
	}Bits;
	uint8_t val;
}RTCReg2;


typedef	union 
{
	struct 
	{
		uint8_t Day : 3; // Present week day
		uint8_t VBAT_EN : 1; // Battery enable bit
		uint8_t PWRFAIL : 1; // Battery supply on
		uint8_t OSC_ON : 1; // Oscillator ON
		uint8_t : 2;
	}Bits;

	uint8_t val;
}RTCReg3;

typedef	union
{
	struct
	{
		uint8_t Date : 4;
		uint8_t Datex10: 2; 
		uint8_t : 2;
	}Bits;
	uint8_t val;
}RTCReg4;

typedef	union
{
	struct
	{
		uint8_t Month : 4; 
		uint8_t Monthx10 : 1; 
		uint8_t Leap : 1; 
		uint8_t : 2;
	}Bits;
	uint8_t val;
}RTCReg5;

typedef	union
{
	struct
	{
		uint8_t Year : 4;
		uint8_t Yearx10 : 4;
	}Bits;
	uint8_t val;
}RTCReg6;


typedef	union
{
	struct 
	{
		uint8_t RS : 3; // Internal divider for 32768Hz oscillator
		uint8_t EXTOSC : 1; // Enable external 32768Hz signal clock
		uint8_t ALM : 2; // Active alarms
		uint8_t SQW_EN : 1; // Enable divided output from de xtal oscillator
		uint8_t MFP_OUT : 1; // State of MFP output
	}Bits;
	uint8_t val;
}RTCreg7;

typedef struct
{
	RTCReg0 Reg0;
	RTCReg1 Reg1;
	RTCReg2 Reg2;
	RTCReg3 Reg3;
	RTCReg4 Reg4;
	RTCReg5 Reg5;
	RTCReg6 Reg6;
	RTCreg7 Reg7;
}RTCCtrl;

// **************************************************************************************
//                  FUNCTION PROTOTYPES 
// **************************************************************************************

uint32_t fWrSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes);
uint32_t fRdSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes);
uint32_t fWrE2prom(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes);
uint32_t fRdE2prom(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes);
uint32_t fRdCtrlReg(uint8_t *data);
uint32_t fRdStatReg(uint8_t *data);
uint32_t fWrCtrlReg(uint8_t *data);
uint32_t fWrStatReg(uint8_t *data);
uint32_t fSetTime(struct tm *DTime);
uint32_t fSetDate(struct tm *DTime);
uint32_t fGetTime(struct tm *DTime);
uint32_t fGetDate(struct tm *DTime);
uint32_t fWriteID(uint8_t *pData);
uint32_t fReadID48(uint8_t *pData);
uint32_t fReadUserSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes);
uint32_t fWriteUserSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes);
uint32_t fWriteE2prom(uint8_t *pData, uint16_t nbytes);
uint32_t fReadE2prom(uint8_t *pData, uint16_t nbytes);
uint32_t fEraseE2Prom(void);
uint32_t fClearCalReg(void);
uint32_t Init_RTC(void);
#endif
