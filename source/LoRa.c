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
 * file       HopeDuino_LoRa.cpp
 * brief      driver for RFM92/95/96/98
 * hardware   HopeRF's RFduino TRx & with HopeRF's LoRa COB rf-module  
 *            
 *
 * version    1.0
 * date       Jun 3 2014
 * author     QY Ruan
 */

#include "MK22F12810.h"
#include "user.h"
#include "LoRa.h"

#define BANDWITH 	125			// in KHz
#define FREQUENCY	868000		// in KHz
#define DEVIATION	35		 	//35KHz for devation
#define SYMBOL_TM	104166  	//9.6Kbps

loraClass RFmodule;
stLoraRunParam LoraRunParam;


/**********************************************************
 **Name:     vInitialize
 **Function: initialize rfm9x
 **Input:    none
 **Output:   none
 **********************************************************/

void vInitialize(void) {

	ClrPOR();
	delay_us(200);
	PORIn();

	// DIO0In();
	vSpiInit(4000000);									//init SPI
	RFmodule.OutputPower = 20;
	RFmodule.Frequency = 868000;
	RFmodule.FrequencyValue.Freq = (FREQUENCY << 11) / 125;		//calc frequency
	RFmodule.BitRateValue = (SYMBOL_TM << 5) / 1000;			//calc bitrate
	RFmodule.DevationValue = (DEVIATION << 11) / 125;			//calc deviation
	RFmodule.BandWidthValue = bSelectBandwidth(BANDWITH);
	RFmodule.PreambleLength = 12;
	RFmodule.SFSel = SF9;
	RFmodule.Modulation = LORA;
	RFmodule.COB = RFM95;
	RFmodule.BWSel = BW125K;
	RFmodule.CRSel = CR4_5;
	RFmodule.FixedPktLength = false;
	RFmodule.PayloadLength = 21;
	RFmodule.CrcDisable = false;


	switch (RFmodule.SFSel) {
	case SF6:
		RFmodule.SFValue = 6;
		break;
	case SF7:
		RFmodule.SFValue = 7;
		break;
	case SF8:
		RFmodule.SFValue = 8;
		break;
	case SF10:
		RFmodule.SFValue = 10;
		break;
	case SF11:
		RFmodule.SFValue = 11;
		break;
	case SF12:
		RFmodule.SFValue = 12;
		break;
	case SF9:
	default:
		RFmodule.SFValue = 9;
		break;
	}
	switch (RFmodule.BWSel) {
	case BW62K:
		RFmodule.BWValue = 6;
		break;				//for RFM95/96/97/98
	case BW250K:
		RFmodule.BWValue = 8;
		break;
	case BW500K:
		RFmodule.BWValue = 9;
		break;
	case BW125K:
	default:
		RFmodule.BWValue = 7;
		break;
	}
	switch (RFmodule.CRSel) {
	default:
	case CR4_5:
		RFmodule.CRValue = 1;
		break;
	case CR4_6:
		RFmodule.CRValue = 2;
		break;
	case CR4_7:
		RFmodule.CRValue = 3;
		break;
	case CR4_8:
		RFmodule.CRValue = 4;
		break;
	}

	if ((RFmodule.SFValue - 4) >= RFmodule.BWValue)
		RFmodule.RsOptimize = true;
	else
		RFmodule.RsOptimize = false;

	vConfig();
	vGoStandby();
}

/**********************************************************
 **Name:     vConfig
 **Function: config rfm9x
 **Input:    none
 **Output:   none
 **********************************************************/
void vConfig(void) {
	uint8_t i, j;
	uint8_t sync;

	vReset();
	vGoStandby();

	//Frequency
	vSpiWrite(((uint16_t) RegFrMsb << 8) + RFmodule.FrequencyValue.freq.FreqH);
	vSpiWrite(((uint16_t) RegFrMid << 8) + RFmodule.FrequencyValue.freq.FreqM);
	vSpiWrite(((uint16_t) RegFrLsb << 8) + RFmodule.FrequencyValue.freq.FreqL);

	//PA Config
	i = bSpiRead(RegPaConfig);
	i &= 0x70;
	if (RFmodule.OutputPower >= 20) {
		i |= 0x0F;
		j = 0x87;
	} else if (RFmodule.OutputPower > 17) {
		i |= (RFmodule.OutputPower - 3 - 2);
		j = 0x87;
	} else if (RFmodule.OutputPower >= 2) {
		i |= (RFmodule.OutputPower - 2);
		j = 0x84;
	} else {
		i |= 0;
		j = 0x84;
	}
	vSpiWrite(((uint16_t) RegPaConfig << 8) + 0x80 + i);	//PA_BOOST
	switch (RFmodule.COB) {
	case RFM92:
	case RFM93:
		vSpiWrite(((uint16_t) RegPaDac_2 << 8) + j);
		break;
	default:
		vSpiWrite(((uint16_t) RegPaDac << 8) + j);
		break;
	}
	j = bSpiRead(RegPaRamp);
	j &= 0x0F;
	j |= bSelectRamping(RFmodule.SymbolTime);
	vSpiWrite(((uint16_t) RegPaRamp << 8) + j);

	//Ocp
	vSpiWrite(((uint16_t) RegOcp << 8) + 0x0F);			//Disable Ocp

	//LNA
	//vSpiWrite(((uint16_t)RegLna<<8)+0x20);		//High & LNA Enable

	// Mode
	i = bSpiRead(RegOpMode);
	j = bSpiRead(RegPaRamp);		//for RFM95/96/97/98
	if (RFmodule.Modulation == LORA) {
		i &= 0x87;
		switch (RFmodule.COB) {
		case RFM96:
		case RFM98:
			i |= 0x08;
			break;
		default:
			break;
		}
		vSpiWrite(((uint16_t) RegOpMode << 8) + i);

		i = bSpiRead(0x31);			//SetNbTrigPeaks
		i &= 0xF8;
		if (RFmodule.SFSel == SF6) {
			i |= 0x05;
			vSpiWrite(0x3100 + i);
			vSpiWrite(0x3700 + 0x0C);
			RFmodule.FixedPktLength = true;			//SF6 must be ImplicitHeaderMode
		} else {
			i |= 0x03;
			vSpiWrite(0x3100 + i);
		}

		switch (RFmodule.COB) {
		uint8_t tmp;
	case RFM92:
	case RFM93:
		if (RFmodule.BWValue > 6)
			tmp = RFmodule.BWValue - 7;
		else
			tmp = 0;
		tmp <<= 6;					//BandWidth
		tmp |= (RFmodule.CRValue << 3);
		if (RFmodule.FixedPktLength)			//ImplicitHeader
			tmp |= 0x04;
		if (!RFmodule.CrcDisable)				//
			tmp |= 0x02;
		if (RFmodule.RsOptimize)		//mandated for when the symbol length exceeds 16ms
			tmp |= 0x01;
		vSpiWrite(((uint16_t) RegModemConfig1 << 8) + tmp);
		tmp = (RFmodule.SFValue << 4);			//SF rate
		tmp |= (0x04 + 0x03);			//AGC ON & Max timeout
		vSpiWrite(((uint16_t) RegModemConfig2 << 8) + tmp);
		break;
	case RFM95:
	case RFM97:
	case RFM96:
	case RFM98:
	default:
		tmp = (RFmodule.BWValue << 4) + (RFmodule.CRValue << 1);
		if (RFmodule.FixedPktLength)
			tmp |= 0x01;
		vSpiWrite(((uint16_t) RegModemConfig1 << 8) + tmp);
		tmp = (RFmodule.SFValue << 4);
		if (!RFmodule.CrcDisable)
			tmp |= 0x04;
		tmp += 0x03;
		vSpiWrite(((uint16_t) RegModemConfig2 << 8) + tmp);
		tmp = 0x04;				//AGC ON
		if (RFmodule.RsOptimize)		//mandated for when the symbol length exceeds 16ms
			tmp |= 0x08;
		vSpiWrite(((uint16_t) RegModemConfig3 << 8) + tmp);
		break;
		}
		vSpiWrite(((uint16_t) RegSymbTimeoutLsb << 8) + 0xFF);//RegSymbTimeoutLsb Timeout = 0x3FF(Max)
		vSpiWrite(
				((uint16_t) RegPreambleMsb_LR << 8)	+ (uint8_t) (RFmodule.PreambleLength >> 8));
		vSpiWrite(
				((uint16_t) RegPreambleLsb_LR << 8) + (uint8_t) RFmodule.PreambleLength);

		vSpiWrite(((uint16_t) RegDioMapping2 << 8) + 0x40);	//RegDioMapping2	DIO5=00(ModeReady), DIO4=01(PllLock)
	}
	else
	{
		switch (RFmodule.COB) {
		case RFM92:
		case RFM93:
			i &= MOUDLE_MASK_2;
			switch (RFmodule.Modulation) {
			case OOK:
				i |= OokMode + (Shaping << 3);
				break;
			case GFSK:
				i |= FskMode + (Shaping << 3);
				break;
			case FSK:
			default:
				i |= FskMode;
				break;
			}
			break;
		default:
			i &= MOUDLE_MASK_1;
			j &= 0x9F;
			switch (RFmodule.Modulation) {
			case OOK:
				i |= OokMode;
				j |= (Shaping << 5);
				break;
			case GFSK:
				i |= FskMode;
				j |= (Shaping << 5);
				break;
			case FSK:
			default:
				i |= FskMode;
				break;
			}
			break;
		}
		vSpiWrite(((uint16_t) RegOpMode << 8) + i);
		vSpiWrite(((uint16_t) RegPaRamp << 8) + j);

		//BitRate
		vSpiWrite(
				((uint16_t) RegBitrateMsb << 8)
						+ (uint8_t) (RFmodule.BitRateValue >> 8));
		vSpiWrite(((uint16_t) RegBitrateLsb << 8) + (uint8_t) RFmodule.BitRateValue);

		//Deviation

		vSpiWrite(
				((uint16_t) RegFdevMsb << 8)
						+ (((uint8_t) (RFmodule.DevationValue >> 8)) & 0x3F));
		vSpiWrite(
				((uint16_t) RegFdevLsb << 8)
						+ (uint8_t) (RFmodule.DevationValue & 0xFF));

		//RxConfig
		vSpiWrite(((uint16_t) RegRxConfig << 8) + AGC_ON + RX_TRIGGER);

		//RxBw
		vSpiWrite(((uint16_t) RegRxBw << 8) + RFmodule.BandWidthValue);

		//OOK
		vSpiWrite(((uint16_t) RegOokPeak << 8) + 0x20 + (0x02 << 3) + 0x00);

		//PreambleDetect
		vSpiWrite(
				((uint16_t) RegPreambleDetect << 8) + PREAMBLE_DECT_ON + PREAMBLE_DECT_3BYTE);
		vSpiWrite(
				((uint16_t) RegPreambleMsb << 8) + (uint8_t) (RFmodule.PreambleLength >> 8));
		vSpiWrite(((uint16_t) RegPreambleLsb << 8) + (uint8_t) RFmodule.PreambleLength);

		//Osc
		vSpiWrite(((uint16_t) RegOsc << 8) + 0x07);		//Close OscClk Output

		//SyncConfig
		if (RFmodule.SyncLength == 0)
			sync = 0;
		else
			sync = RFmodule.SyncLength - 1;
		vSpiWrite(
				((uint16_t) RegSyncConfig << 8) + AUTO_RST_RX_ONwPLL + SYNC_ON
						+ (sync & 0x07));
		for (i = 0; i < 8; i++)								//SyncWordSetting
			vSpiWrite(((uint16_t) (RegSyncValue1 + i) << 8) + RFmodule.SyncWord[i]);

		i = DcFree_NRZ + AddrFilter_NONE + CrcCalc_CCITT;
		if (!RFmodule.FixedPktLength)
			i += VariablePacket;
		if (!RFmodule.CrcDisable)
			i += CrcOn;
		vSpiWrite(((uint16_t) RegPacketConfig1 << 8) + i);
		vSpiWrite(((uint16_t) RegPacketConfig2 << 8) + PacketMode);

		if (RFmodule.FixedPktLength)								//Set Packet length
			vSpiWrite(((uint16_t) RegPayloadLength << 8) + RFmodule.PayloadLength);
		else
			vSpiWrite(((uint16_t) RegPayloadLength << 8) + 0xFF);

		vSpiWrite(((uint16_t) RegFifoThresh << 8) + 0x01);
		vSpiWrite(((uint16_t) RegDioMapping2 << 8) + 0x61);	//DIO4 PllLock / DIO5 Data / PreambleDetect
	}
}

/**********************************************************
 **Name:     vGoRx
 **Function: set rf9x to receive mode
 **Input:    none
 **Output:   none
 **********************************************************/
void vGoRx(void) {
	uint8_t tmp;
	if (RFmodule.Modulation == LORA) {
		if (RFmodule.FixedPktLength)								//Set Packet length
			vSpiWrite(((uint16_t) RegPayloadLength_LR << 8) + RFmodule.PayloadLength);
		else
			vSpiWrite(((uint16_t) RegPayloadLength_LR << 8) + 0xFF);

		vSpiWrite(((uint16_t) RegDioMapping1 << 8) + 0x22);	//DIO0 RxDone / DIO1 CadDetected / DIO2 FhssChangeChannel /DIO3 PayloadCrcError
		vSpiWrite(((uint16_t) RegIrqFlags << 8) + AllIrqMask);//Clear All Interrupt
		vSpiWrite(((uint16_t) RegIrqFlagsMask << 8)	+ (AllIrqMask & (~(RxDoneMask | RxTimeoutMask))));//just enable RxDone & Timeout

		tmp = bSpiRead(RegFifoRxBaseAddr);				//Read RxBaseAddr
		vSpiWrite(((uint16_t) RegFifoAddrPtr << 8) + tmp);//RxBaseAddr -> FiFoAddrPtr��
	} else {
		if (RFmodule.CrcDisable)
			vSpiWrite(((uint16_t) RegDioMapping1 << 8) + 0x00);	//DIO0 PayloadReady / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty
		else
			vSpiWrite(((uint16_t) RegDioMapping1 << 8) + 0x40);	//DIO0 CrcOk  / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty
	}

	tmp = bSpiRead(RegOpMode);
	tmp &= MODE_MASK;
	tmp |= RADIO_RX;
	vSpiWrite(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     vGoStandby
 **Function: set rf9x to standby mode
 **Input:    none
 **Output:   none
 **********************************************************/
void vGoStandby(void) {
	uint8_t tmp;
	tmp = bSpiRead(RegOpMode);
	tmp &= MODE_MASK;
	tmp |= RADIO_STANDBY;
	vSpiWrite(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     vGoSleep
 **Function: set rf9x to sleep mode
 **Input:    none
 **Output:   none
 **********************************************************/
void vGoSleep(void) {
	uint8_t tmp;
	tmp = bSpiRead(RegOpMode);
	tmp &= MODE_MASK;
	tmp |= RADIO_SLEEP;
	vSpiWrite(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     bSendMessage
 **Function: set rf9x to sleep mode
 **Input:    msg------for which message to send
 length---message length
 **Output:   true-----send ok
 false----send error/over time
 **********************************************************/
bool bSendMessage(uint8_t msg[], uint8_t length) {
	uint8_t tmp;

	if(LoraRunParam.bTX_data)
		return false;

	if (RFmodule.Modulation == LORA) {
		vSpiWrite(((uint16_t) RegPayloadLength_LR << 8) + length);
		vSpiWrite(((uint16_t) RegDioMapping1 << 8) + 0x62);								//DIO0 TxDone / DIO1 CadDetected / DIO2 FhssChangeChannel /DIO3 PayloadCrcError
		vSpiWrite(((uint16_t) RegIrqFlags << 8) + AllIrqMask);							//Clear All Interrupt
		vSpiWrite(((uint16_t) RegIrqFlagsMask << 8)	+ (AllIrqMask & (~TxDoneMask)));	//just enable TxDone

		tmp = bSpiRead(RegFifoTxBaseAddr);				//Read TxBaseAddr
		vSpiWrite(((uint16_t) RegFifoAddrPtr << 8) + tmp);//RxBaseAddr -> FiFoAddrPtr��

		vSpiBurstWrite(RegFifo, msg, length);
	} else {
		vSpiWrite(((uint16_t) RegDioMapping1 << 8) + 0x00);	//DIO0 PacketSend  / DIO1 FifoLevel / DIO2 FifoFull /DIO3 FifoEmpty

		if (!RFmodule.FixedPktLength)
			vSpiWrite(((uint16_t) RegFifo << 8) + length);
		vSpiBurstWrite(RegFifo, msg, length);
	}

	tmp = bSpiRead(RegOpMode);
	tmp &= MODE_MASK;
	tmp |= RADIO_TX;
	vSpiWrite(((uint16_t) RegOpMode << 8) + tmp);

	LoraRunParam.bTX_data = true;
	return true;
}

/**********************************************************
 **Name:     bGetMessage
 **Function: check receive packet
 **Input:    msg------for which message to read
 **Output:   packet length
 0--------have no packet
 **********************************************************/
uint8_t bGetMessage(uint8_t msg[]) {
	uint8_t length, status, error;

	if(LoraRunParam.bTX_data)return false;

	LoRa_GetIrqStatus();
	status = (LoraRunParam.IRQstat & 0xf0) & RxDoneMask;
	error = (LoraRunParam.IRQstat & 0xf0) & (PayloadCrcErrorMask | RxTimeoutMask );

	if (status == RxDoneMask && !error)				//Receive CrcOk or PayloadReady
	{
		if (RFmodule.Modulation == LORA)
		{
			LoraRunParam.IRQstat = bSpiRead(RegIrqFlags);			//Read Interrupt Flag for do something
			LoraRunParam.pktSNR = bSpiRead(RegPktSnrValue) / 2; 		// SNR in dB
			LoraRunParam.pktRSSI =bSpiRead(RegPktRssiValue) - 137;	// current RSSI in dBm
			LoraRunParam.RSSI =bSpiRead(RegRssiValue_LR) -137;	// last received packet RSSI

			vSpiWrite(((uint16_t) RegFifoAddrPtr << 8) + bSpiRead(RegFifoRxCurrentAddr));

			if(RFmodule.FixedPktLength)
				length =RFmodule.PayloadLength;
			else
				length = bSpiRead(RegRxNbBytes);

			vSpiBurstRead(RegFifo, msg, length);
			vSpiWrite(((uint16_t) RegIrqFlags << 8) + AllIrqMask);//Clear All Interrupt
		}
		else
		{
			if (RFmodule.FixedPktLength)
				length = RFmodule.PayloadLength;
			else
				length = bSpiRead(RegFifo);
			vSpiBurstRead(RegFifo, msg, length);
		}
		// vGoStandby();
		// vGoRx();
		return (length);
	}
	return (0);
}

/**********************************************************
 **Name:     vReset
 **Function: hardware reset rf69 chipset
 **Input:    none
 **Output:   none
 **********************************************************/
void vReset(void) {
	uint8_t tmp;
	POROut();
	switch (RFmodule.COB) {
	case RFM92:						//High Reset; Normal for Low
	case RFM93:
		ClrPOR();
		delay_us(200);				//at least 100us for reset
		SetPOR();
		break;
	case RFM95:
	case RFM96:
	case RFM97:
	case RFM98:
	default:
		SetPOR();
		delay_us(200);				//at least 100us for reset
		ClrPOR();
		break;
	}
	PORIn();							//set POR for free
	delay_ms(6);						//wait for ready
	ClrPOR();							//note: help for LowReset

	tmp = bSpiRead(RegOpMode);
	tmp &= MODE_MASK;
	tmp |= RADIO_SLEEP;
	vSpiWrite(((uint16_t) RegOpMode << 8) + tmp);

	tmp &= 0x7F;
	if (RFmodule.Modulation == LORA)
		tmp |= 0x80;
	vSpiWrite(((uint16_t) RegOpMode << 8) + tmp);
}

/**********************************************************
 **Name:     bSelectBandwidth
 **Function:
 **Input:    BandWidth
 **Output:   BandWidthValue
 **********************************************************/
uint8_t bSelectBandwidth(uint16_t rx_bw) {
	if (rx_bw <= 10)
		return 0x15;						//10.4KHz 	Min
	else if (rx_bw < 13)
		return 0x0D;						//12.5KHz
	else if (rx_bw < 16)
		return 0x05;						//15.6KHz
	else if (rx_bw < 21)
		return 0x14;						//20.8KHz
	else if (rx_bw <= 25)
		return 0x0C;						//25.0KHz
	else if (rx_bw < 32)
		return 0x04;						//31.3KHz
	else if (rx_bw < 42)
		return 0x13;						//41.7KHz
	else if (rx_bw <= 50)
		return 0x0B;						//50.0KHz
	else if (rx_bw < 63)
		return 0x03;						//62.5KHz
	else if (rx_bw < 84)
		return 0x12;						//83.3KHz
	else if (rx_bw <= 100)
		return 0x0A;						//100KHz
	else if (rx_bw <= 125)
		return 0x02;						//125KHz
	else if (rx_bw < 167)
		return 0x11;						//167KHz
	else if (rx_bw <= 200)
		return 0x09;						//200KHz
	else if (rx_bw <= 250)
		return 0x01;						//250KHz
	else if (rx_bw <= 333)
		return 0x10;						//333KHz
	else if (rx_bw <= 400)
		return 0x08;						//400KHz
	else
		return 0x00;						//500KHz Max
}

/**********************************************************
 **Name:     bSelectRamping
 **Function:
 **Input:    symbol time
 **Output:   ramping value
 **********************************************************/
uint8_t bSelectRamping(uint32_t symbol) {
	uint32_t SymbolRate;

	SymbolRate = symbol / 1000;			//ns->us
	SymbolRate = SymbolRate / 4;			// 1/4 ramping

	if (SymbolRate <= 10)
		return 0x0F;					//10us
	else if (SymbolRate <= 12)
		return 0x0E;					//12us
	else if (SymbolRate <= 15)
		return 0x0D;					//15us
	else if (SymbolRate <= 20)
		return 0x0C;					//20us
	else if (SymbolRate <= 25)
		return 0x0B;					//25us
	else if (SymbolRate <= 31)
		return 0x0A;					//31us
	else if (SymbolRate <= 40)
		return 0x09;					//40us
	else if (SymbolRate <= 50)
		return 0x08;					//50us
	else if (SymbolRate <= 62)
		return 0x07;					//62us
	else if (SymbolRate <= 100)
		return 0x06;					//100us
	else if (SymbolRate <= 125)
		return 0x05;					//125us
	else if (SymbolRate <= 250)
		return 0x04;					//250us
	else if (SymbolRate <= 500)
		return 0x03;					//500us
	else if (SymbolRate <= 1000)
		return 0x02;					//1000us
	else if (SymbolRate <= 2000)
		return 0x01;					//2000us
	else
		return 0x00;
}

/**********************************************************
 **Name:     LoRa_GetStatus
 **Function: Get modem status register
 **Input:
 **Output:   status value
 **********************************************************/

uint8_t LoRa_GetModemStatus(void)
{
	uint8_t value = 0;
	vSpiBurstRead(RegModemStat, &value, 1);
	return value;
}

/**********************************************************
 **Name:     LoRa_GetStatus
 **Function: Read IRQ status and clear IRQs
 **Input:
 **Output:   status value
 **********************************************************/

uint8_t LoRa_GetIrqStatus(void)
{
	vSpiBurstRead(RegIrqFlags, &LoraRunParam.IRQstat, 1);
	// vSpiWrite(((uint16_t) RegIrqFlags << 8) + AllIrqMask); //Clear All Interrupt
	return LoraRunParam.IRQstat;
}

uint32_t LoRaCheckTXstatus(void)
{
	if(LoraRunParam.bTX_data)
	{
		LoRa_GetIrqStatus();
		if (LoraRunParam.IRQstat & TxDoneMask)
		{
			LoraRunParam.bTX_data = false;
			vSpiWrite(((uint16_t) RegIrqFlags << 8) + AllIrqMask);//Clear All Interrupt
			vGoRx();
		}

	}
	return LoraRunParam.bTX_data;
}

