/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: 
 *  (1) "AS IS" WITH NO WARRANTY; 
 *  (2) TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, HopeRF SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) HopeRF
 *
 * website: www.HopeRF.com
 *          www.HopeRF.cn   
 */

/*! 
 * file       HopeDuino_LoRa.h
 * brief      driver for RFM92/95/96/98
 * hardware   HopeDuino with HopeRF's LoRa COB rf-module
 *            
 *
 * version    1.0
 * date       Jun 3 2014
 * author     QY Ruan
 */

#ifndef LORA_h
#define LORA_h

#include "MK22F12810.h"
#include "pin_mux.h"
#include "LoRa_SPI.h"


/**********************************************************
 * Reset pin table define
 **********************************************************/

#define POROut()	(RFM_RST_GPIO->PDDR |= (1U << RFM_RST_GPIO_PIN))
#define	PORIn()		(RFM_RST_GPIO->PDDR &= ~(1U << RFM_RST_GPIO_PIN))
#define SetPOR()	(RFM_RST_GPIO->PSOR = 1U << RFM_RST_GPIO_PIN)
#define	ClrPOR()	(RFM_RST_GPIO->PCOR = 1U << RFM_RST_GPIO_PIN)
#define DIO0_H()	0

/**********************************************************
 **RF69 Register define
 **********************************************************/
//Common Register

#define 	RegFifo 			0x00
#define 	RegOpMode 			0x01
#define 	RegFrMsb 			0x06
#define 	RegFrMid 			0x07
#define 	RegFrLsb 			0x08
#define		RegPaConfig			0x09
#define 	RegPaRamp 			0x0A
#define 	RegOcp 			    0x0B
#define 	RegLna 			    0x0C

#define 	RegDioMapping1 	    0x40
#define 	RegDioMapping2 	    0x41
#define 	RegVersion 		    0x42

//for 6/7/8
#define		RegPllHop			0x44
#define		RegTcxo				0x4B
#define		RegPaDac			0x4D
#define		RegFormerTemp		0x5B
#define		RegBitrateFrac		0x5D
#define		RegAgcRef			0x61
#define		RegAgcThresh1		0x62
#define		RegAgcThresh2		0x63
#define		RegAgcThresh3		0x64
#define		RegPll				0x70

//for 2/3
#define		RegAgcRef_2			0x43
#define		RegAgcThresh1_2		0x44
#define		RegAgcThresh2_2		0x45
#define		RegAgcThresh3_2		0x46
#define		RegPllHop_2			0x4B
#define		RegTcxo_2			0x58
#define		RegPaDac_2			0x5A
#define		RegPll_2			0x5C
#define		RegPllLowPn			0x5E
#define		RegFormerTemp_2		0x6C
#define		RegBitrateFrac_2	0x70

//ASK/FSK/GFSK Registers

#define 	RegBitrateMsb 		0x02
#define 	RegBitrateLsb 		0x03
#define 	RegFdevMsb 		    0x04
#define 	RegFdevLsb 		    0x05

#define		RegRxConfig			0x0D
#define 	RegRssiConfig 		0x0E
#define		RegRssiCollision	0x0F
#define 	RegRssiThresh 		0x10
#define 	RegRssiValue 		0x11
#define 	RegRxBw 			0x12
#define 	RegAfcBw 			0x13
#define 	RegOokPeak 		    0x14
#define 	RegOokFix 			0x15
#define 	RegOokAvg 			0x16
#define 	RegAfcFei 			0x1A
#define 	RegAfcMsb 			0x1B
#define 	RegAfcLsb 			0x1C
#define 	RegFeiMsb 			0x1D
#define 	RegFeiLsb 			0x1E
#define		RegPreambleDetect	0x1F
#define 	RegRxTimeout1 		0x20
#define 	RegRxTimeout2 		0x21
#define 	RegRxTimeout3 		0x22
#define		RegRxDelay			0x23
#define 	RegOsc 				0x24
#define 	RegPreambleMsb 	    0x25
#define 	RegPreambleLsb 	    0x26
#define 	RegSyncConfig 		0x27
#define 	RegSyncValue1		0x28
#define 	RegSyncValue2       0x29
#define 	RegSyncValue3       0x2A
#define 	RegSyncValue4       0x2B
#define 	RegSyncValue5       0x2C
#define 	RegSyncValue6       0x2D
#define 	RegSyncValue7       0x2E
#define 	RegSyncValue8       0x2F
#define 	RegPacketConfig1 	0x30
#define 	RegPacketConfig2 	0x31
#define 	RegPayloadLength 	0x32
#define 	RegNodeAdrs 		0x33
#define 	RegBroadcastAdrs 	0x34
#define 	RegFifoThresh 		0x35
#define		RegSeqConfig1		0x36
#define		RegSeqConfig2		0x37
#define		RegTimerResol		0x38
#define		RegTimer1Coef		0x39
#define		RegTimer2Coef		0x3A
#define		RegImageCal			0x3B
#define		RegTemp				0x3C
#define		RegLowBat			0x3D
#define 	RegIrqFlags1 		0x3E
#define 	RegIrqFlags2 		0x3F

//LoRa Register

#define		RegFifoAddrPtr				0x0D
#define 	RegFifoTxBaseAddr			0x0E
#define		RegFifoRxBaseAddr			0x0F
#define 	RegFifoRxCurrentAddr		0x10
#define 	RegIrqFlagsMask				0x11
#define 	RegIrqFlags					0x12
#define 	RegRxNbBytes				0x13
#define 	RegRxHeaderCntValueMsb    	0x14
#define 	RegRxHeaderCntValueLsb		0x15
#define 	RegRxPacketCntValueMsb		0x16
#define 	RegRxPacketCntValueLsb		0x17
#define 	RegModemStat				0x18
#define 	RegPktSnrValue				0x19
#define 	RegPktRssiValue				0x1A
#define 	RegRssiValue_LR				0x1B
#define		RegHopChannel				0x1C
#define 	RegModemConfig1				0x1D
#define 	RegModemConfig2 			0x1E
#define 	RegSymbTimeoutLsb 			0x1F
#define		RegPreambleMsb_LR			0x20
#define 	RegPreambleLsb_LR			0x21
#define 	RegPayloadLength_LR 	    0x22
#define		RegMaxPayloadLength			0x23
#define		RegHopPeriod				0x24
#define		RegFifoRxByteAddr			0x25
#define		RegModemConfig3				0x26		// only for 6/7/8

/**********************************************************
 **RF69 mode status
 **********************************************************/
#define		RADIO_SLEEP			(0x00)
#define		RADIO_STANDBY		(0x01)
#define		RADIO_TX			(0x03)
#define		RADIO_RX			(0x05)

#define		FskMode				(0<<5)
#define		OokMode				(1<<5)

#define		Shaping				2

#define 	MODE_MASK			0xF8

#define		MOUDLE_MASK_2		0x87			//for RFM92/93
#define		MOUDLE_MASK_1		0x9F			//for RFM95/96/97/98

#define		AFC_ON				(1<<4)
#define		AGC_ON				(1<<3)
#define		RX_TRIGGER			0x06

#define		PREAMBLE_DECT_ON	(1<<7)
#define		PREAMBLE_DECT_1BYTE	(0<<5)
#define		PREAMBLE_DECT_2BYTE	(1<<5)
#define		PREAMBLE_DECT_3BYTE	(2<<5)

#define		AUTO_RST_RX_OFF		(0<<6)
#define		AUTO_RST_RX_ON		(1<<6)
#define		AUTO_RST_RX_ONwPLL	(2<<6)
#define		SYNC_ON				(1<<4)

//for PacketConfig
#define		VariablePacket		(1<<7)
#define		DcFree_NRZ			(0<<5)
#define		DcFree_MANCHESTER	(1<<5)
#define		DcFree_WHITENING	(2<<5)
#define		CrcOn				(1<<4)
#define		CrcDisAutoClear		(1<<3)
#define		AddrFilter_NONE		(0<<1)
#define		AddrFilter_NODE		(1<<1)
#define		AddrFilter_ALL		(2<<1)
#define		CrcCalc_CCITT		0x00
#define		CrcCalc_IBM			0x01

#define		PacketMode			(1<<6)
#define		ContinuousMode		(0<<6)

//for LoRa
#define		AllIrqMask				0xFF
#define		RxTimeoutMask			(1<<7)
#define		RxDoneMask				(1<<6)
#define		PayloadCrcErrorMask		(1<<5)
#define		ValidHeaderMask			(1<<4)
#define		TxDoneMask				(1<<3)
#define		CadDoneMask				(1<<2)
#define		FhssChangeChannelMask	(1<<1)
#define		CadDetectedMask			(1<<0)

typedef union tag_FreqStruct{
	struct {
		uint32_t FreqL :8;
		uint32_t FreqM :8;
		uint32_t FreqH :8;
		uint32_t FreqX :8;
	} freq;
	unsigned long Freq;
} FreqStruct;

typedef enum tag_modulationType {
	OOK, FSK, GFSK, LORA
}modulationType;

typedef enum tag_moduleType {
	RFM92, RFM93, RFM95, RFM96, RFM97, RFM98
}moduleType;

typedef enum tag_sfType {
	SF6, SF7, SF8, SF9, SF10, SF11, SF12
}sfType;

typedef enum tag_bwType {
	BW62K, BW125K, BW250K, BW500K
}bwType;
//

typedef enum tag_crType {
	CR4_5, CR4_6, CR4_7, CR4_8
}crType;

typedef struct tag_loraClass
{
	modulationType Modulation;					//OOK/FSK/GFSK/LORA
	moduleType COB;								//Chip on board

	//common parameter
	
	uint32_t Frequency;//unit: KHz
	uint8_t OutputPower;//unit: dBm   range: 2-20 [2dBm~+20dBm]
	uint16_t PreambleLength;//unit: byte

	bool FixedPktLength;//OOK/FSK/GFSK:
						//	 	true-------fixed packet length
						//   	false------variable packet length
						//LoRa:
						//	 	true-------implicit header mode
						//      false------explicit header mode

	bool CrcDisable;//OOK/FSK/GFSK:
					//		true-------CRC disable
					//		fasle------CRC enable with CCITT
					//LoRa:
					//		true-------Header indicates CRC off
					//		false------Header indicates CRC on
	uint8_t PayloadLength;//PayloadLength is need to be set a value, when FixedPktLength is true.

	//for OOK/FSK/GFSK parameter
	
	uint32_t SymbolTime;//unit: ns
	uint32_t Devation;//unit: KHz
	uint16_t BandWidth;//unit: KHz
	uint8_t SyncLength;//unit: none, range: 1-8[Byte], value '0' is not allowed!
	uint8_t SyncWord[8];

	//for LoRa parameter
	
	sfType SFSel;//unit: none, range: SF6~SF12
	bwType BWSel;
	crType CRSel;
	FreqStruct FrequencyValue;
	uint16_t BitRateValue;
	uint16_t DevationValue;
	uint8_t BandWidthValue;
	uint8_t SFValue;
	uint8_t BWValue;
	uint8_t CRValue;
	bool RsOptimize;
}loraClass;

typedef struct tag_LoraRunParam
{
	bool bTX_data;		// Transmission on process
	uint8_t IRQstat;
	uint8_t pktSNR;		// Estimation of SNR on last packet received
	uint8_t pktRSSI;	// RSSI of the latest packet received dBm
	uint8_t RSSI;		// Current RSSI value dBm
}stLoraRunParam;

/* Function declaration */

uint8_t bSelectBandwidth(uint16_t rx_bw);
uint8_t bSelectRamping(uint32_t symbol);
bool bSendMessage(uint8_t msg[], uint8_t length);
uint8_t bGetMessage(uint8_t msg[]);
void vReset(void);
void vInitialize(void);
void vConfig(void);
void vGoRx(void);
void vGoStandby(void);
void vGoSleep(void);
uint8_t LoRa_GetModemStatus(void);
uint8_t LoRa_GetIrqStatus(void);
uint32_t LoRaCheckTXstatus(void);

#else
#warning "LoRa.h have been defined!"

#endif
