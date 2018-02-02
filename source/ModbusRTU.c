/*
 * ModbusRTU.c
 *
 * Created: 08/02/2016 8:56:06
 *  Author: agarcia
 */ 

#include <string.h>
#include "MK22F12810.h"
// #include "fsl_uart.h"
#include "fsl_clock.h"
#include "user.h"
#include "ModbusRTU.h"
#include "SunPos.h"
#include "LoRa.h"
#include "LIS2HH12.h"
#include "Control.h"


/*uart_handle_t g_uartHandle;
uart_config_t user_config;
uart_transfer_t sendXfer;
uart_transfer_t receiveXfer;
volatile bool txFinished = false;
volatile bool rxFinished = false;
uint8_t ringBuffer[RING_BUFFER_SIZE];*/

uint8_t Test_String[] = " Hello stupid world!\n";
uint8_t Test[] = "UUUUUUUUUUUU";

bool SendEnable = false;

stMbusReadRSP stReadRSP;
stMbusWriteRSP stWRSP;

stSerialPort stCom[1];	// struct for manage COM serial ports

const stCOM_Cfg stCOM_DfltCfg=
{
	.Mbus_sID = 1,
	.Timeout = 10,
	.BaudRate = 115200,
	.Ctrl =
	{
		.MR_Bits =
			{
			.Chr_Length = 0,
			.Parity = 0,
			.Stop_bits = 0,
			}
	}
};


stCOM_Cfg stCOMcfg;

uint32_t Mbus_State = 0;
uint32_t Expiration_tm = 0;

volatile bool bRS4xx_Send;

const uint8_t hiden_pass[8] = {'*', '*', '*', '*', '*', '*', '*', '*'};
const uint8_t MasterPass[8] = BACKDOOR_PASS;

/*  Modbus USART function implementation */

void UART1_RX_TX_DriverIRQHandler(void)
{
	// Read S1 to clear interrupt flags
	uint32_t status = UART1->S1;

	if(status & UART_S1_RDRF_MASK)
	{
		stCom[0].Buffer[stCom[0].RX_bytes++] = UART1->D;

		if(stCom[0].RX_bytes == (COM_BUFFER -1))
		{
			stCom[0].bRX_BufferFull = true;
		}
		else if(stCom[0].RX_bytes > (COM_BUFFER -1))
		{
			stCom[0].bOverrun=true;
			stCom[0].RX_bytes = 0;
		}

		stCom[0].lastCom_tm = GetTickCount();
	}

	if(stCom[0].bTX_InProgress && (status & UART_S1_TDRE_MASK))
	{
		if(stCom[0].TX_bytes)
		{
			stCom[0].TX_bytes--;
			UART1->D = *(stCom[0].pTX_Data++);
		}
		else
		{
			UART1->C2 &= ~UART_C2_TCIE_MASK;					// Disable interrupts for Tx complete
			UART1->C2 |= UART_C2_RE_MASK;						// Enable receiver
			stCom[0].bTX_InProgress = false;
		}
	}

}

inline uint16_t swap16(uint16_t n)
{
	return (n >> 8) | (n << 8);
}

uint32_t USART_SEND(uint8_t* pData,uint8_t nbytes, uint8_t port)
{
	if(port == LORA_PORT)
	{
		return bSendMessage(pData, nbytes);
	}
	else if(!stCom[port].bTX_InProgress && nbytes)
	{
		UART1->C2 &= ~UART_C2_RE_MASK; 	// Disable receiver

		stCom[port].TX_bytes = nbytes;
		stCom[port].pTX_Data = pData;
		stCom[port].bTX_InProgress = 1;
		stCom[0].TX_bytes--;
		UART1->D = *(stCom[port].pTX_Data++);
		UART1->C2 |= UART_C2_TCIE_MASK;					// Enable interrupts for Tx complete
		return 1;
	}

	return 0;
}

void Configure_COM_Dflt()
{
	Configure_COM((stCOM_Cfg *)&stCOM_DfltCfg);
}

void Configure_COM(stCOM_Cfg *pCfg)
{

	memcpy((uint8_t *)&stCOMcfg, pCfg, sizeof(stCOM_Cfg));

}

void uart_init (UART_Type *uartch, int sysclk, int baud)
{
    register uint16_t sbr, brfa;
    uint32_t brfd;
    uint8_t temp;

    /* Enable the clock to the selected UART */

    if(uartch == (UART_Type *)UART0_BASE)
        SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
    else
        if (uartch == (UART_Type *)UART1_BASE)
            SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
        else
            if (uartch == (UART_Type *)UART2_BASE)
                SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;

    /* Make sure that the transmitter and receiver are disabled while we
     * change settings.
     */

    uartch->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

    /* Configure the UART for 8-bit mode, no parity */
    /* We need all default settings, so entire register is cleared */

    uartch->C1 = 0;


    /* Calculate baud settings */

    sbr = (uint16_t)(sysclk /(baud * 16));

    /* Save off the current value of the UARTx_BDH except for the SBR */

    temp = uartch->BDH & ~(UART_BDH_SBR(0x1F));
    uartch->BDH |= (temp | UART_BDH_SBR(sbr >>8));
    uartch->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);

    /* Determine if a fractional divider is needed to get closer to the baud rate */
    brfd = (sysclk % (baud * 16)) / (sysclk / 32);
    brfa = (uint16_t)brfd;

    /* Save off the current value of the UARTx_C4 register except for the BRFA */

    temp = uartch->C4 & ~(UART_C4_BRFA(0x1F));
    uartch->C4 = temp |  UART_C4_BRFA(brfa);

    /* Enable receiver and transmitter */
    uartch->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );

    Configure_COM_Dflt();
}

void USART_Init(void)
{
	uart_init (UART1, CLOCK_GetFreq(kCLOCK_CoreSysClk), 115200);

	UART1->MODEM |= UART_MODEM_TXRTSE_MASK | UART_MODEM_TXRTSPOL_MASK;	// Enable RTS for rs485 mode
	UART1->C2 |=  UART_C2_RIE_MASK;					// Enable interrupts for Rx an Tx complete

	NVIC_SetPriority(UART1_RX_TX_IRQn, 3);
	NVIC_EnableIRQ(UART1_RX_TX_IRQn);

	stCom[0].Error_cnt=0;
	stCom[0].lastCom_tm=GetTickCount();
}

uint16_t fReadHldRegRSP(uint8_t* pSt,uint16_t start, uint16_t max_legth, stMbusReadRSP *pRSP, uint8_t port)
{
	int16_t x = start, y=0;

	if((pRSP->nBytes > MB_MAX_DATALENGTH*2) || ((start + pRSP->nBytes) > max_legth))return ILLEGAL_DATA_ADDRESS;

	do
	{
		pRSP->Data[y]=*(pSt+x+1);
		pRSP->Data[y+1]=*(pSt+x);
		x+=2;
		y+=2;
	}while (y < pRSP->nBytes);

	x = crc16((uint8_t*)pRSP , y + 3);

	pRSP->Data[y++] = (uint8_t)x;
	pRSP->Data[y++] = (uint8_t)(x >> 8);

	USART_SEND((uint8_t*)pRSP ,y + 3 ,port);
	return NO_ERROR;
}

uint16_t fWriteHldRegRSP(uint8_t* pSt,uint8_t *pData, uint16_t max_legth, stMbusWriteRSP *pstWRSP, uint8_t port)
{
	uint16_t sRegister, nBytes;


	nBytes= 2 * pstWRSP->nRegisters;
	sRegister = 2 * (pstWRSP->Start & 0x00ff);

	pstWRSP->Start=swap16(pstWRSP->Start);
	pstWRSP->nRegisters=swap16(pstWRSP->nRegisters);
	pstWRSP->CRC=crc16((uint8_t*)pstWRSP,6);

	if((sRegister > MB_MAX_DATALENGTH*2) || (sRegister+nBytes > max_legth))
		return ILLEGAL_DATA_ADDRESS;

	if(pSt != NULL)
	{
		pSt += sRegister;

		for(uint16_t i=0; i<nBytes; i+=2)
		{
			*(pSt+i)=*(pData+i+1);
			*(pSt+i+1)=*(pData+i);
		}
	}
	return NO_ERROR;
}

uint32_t Compute_MBUSRequest(uint8_t *pData, uint8_t port, uint8_t Slave)
{
	uint16_t x,y;
	uint16_t Start, sRegister, nRegister, iCRC;
	uint8_t rGroup, Mb_Error = NO_ERROR;

	x=1;

	stReadRSP.SlaveID = *pData;
	if((stReadRSP.SlaveID != Slave) && stReadRSP.SlaveID)return SLAVE_NOT_ME;

	stReadRSP.Function=*(pData+x);
	x++;
	Start = swap16(*((uint16_t *)(pData + x)));
	// Start = (*(pData+x) << 8) | *(pData+x+1);
	x+=2;

	nRegister = swap16(*((uint16_t *)(pData + x)));
	// nRegister=(*(pData+x)<< 8) | *(pData+x+1);
	x+=2;

	sRegister=(Start & 0x00ff) <<1;
	rGroup=(uint8_t)(Start>>8);

	if(stReadRSP.Function==FUNC_READ_HOLDING_REGISTERS || stReadRSP.Function==FUNC_READ_INPUT_REGISTERS)
	{
        iCRC=(*(pData+x+1)<<8) | *(pData+x) ;

        stReadRSP.nBytes=(uint8_t)(nRegister<<1);

        if(iCRC!=crc16((uint8_t *)pData,6))
        {
            Mb_Error = MEMORY_PARITY_ERROR;
        }
		else
		{
			switch(rGroup)
			{
				/* Slave and analog data sensor registers */

				case REG_RAW:
					Mb_Error=fReadHldRegRSP((uint8_t *)&GetSunParam()->iElevacion, sRegister, sizeof(stSunP) - offsetof(stSunP, iElevacion), &stReadRSP,port);
					break;
				case REG_DATETIME:
					Mb_Error=fReadHldRegRSP((uint8_t *)GetSunParam()->datetime, sRegister, sizeof(struct tm), &stReadRSP,port);
					break;
				case REG_LORA:
					break;
				default:
					Mb_Error=ILLEGAL_DATA_ADDRESS;
					break;
			}
		}
	}
	else if(stReadRSP.Function==FUNC_WRITE_MULTIPLE_REGISTERS)
	{
        stReadRSP.nBytes=*(pData+x);

        y=x+stReadRSP.nBytes;			// End of data
        y++;
		x++;

		stWRSP.SlaveID=stReadRSP.SlaveID;
		stWRSP.Function=stReadRSP.Function;
		stWRSP.Start=Start;
		stWRSP.nRegisters=nRegister;
		stWRSP.CRC = 0;

        iCRC=(*(pData+y+1)<<8) |*(pData+y);

        if(iCRC!=crc16((uint8_t *)pData,y))
        {
			Mb_Error = MEMORY_PARITY_ERROR;
        }
		else
		{
			float fparam;

				switch(rGroup)
				{
					stModBusCmd Input;

					case REG_CFG_RAW:
						// Mb_Error=fWriteHldRegRSP((uint8_t *)GetSysConfig(), pData+x, offsetof(stNV_Data, Checksum), &stWRSP,port);
						break;
					case REG_CMD:
						Mb_Error = fWriteHldRegRSP((uint8_t *)&Input, pData+x, sizeof(stModBusCmd), &stWRSP, port);
						switch(Input.cmd)
						{
							case 1:
								fparam = Input.param[0] << 16 | Input.param[1];
								GetMotorPIDCtrl()[0].target = fparam;
								GetMotorDrive()->status.bits.Auto = 0;
								GetMotorDrive()->status.bits.update = 1;
								break;
							case 2:
								GetMotorDrive()->command = eDRVCMD_STOP;
								break;
							case 3:
								GetMotorDrive()->status.bits.Auto = 1;
								break;
							case 4:
								if(strncmp((void *)Input.param, "CAL", 3) == 0)
									CaptureRefVector();
								else
									Mb_Error = ILLEGAL_DATA_VALUE;
								break;
							default:
								Mb_Error = ILLEGAL_DATA_VALUE;
								break;
						}
						break;
					default:
						Mb_Error=ILLEGAL_DATA_ADDRESS;
						break;
				}

			if(Mb_Error == NO_ERROR)
			{
				USART_SEND((uint8_t *)&stWRSP, sizeof(stMbusWriteRSP), port);	// Send MBus write request response
			}
		}
	}

	if(Mb_Error != NO_ERROR && Mb_Error != MEMORY_PARITY_ERROR)
	{
		uint8_t MB_exception[5];

		MB_exception[0]=stReadRSP.SlaveID;
		MB_exception[1]=stReadRSP.Function | 0x80;	// Function code Adding 0x80 results in exception code
		MB_exception[2]=Mb_Error;
		x = crc16((uint8_t*)&MB_exception,3);
		MB_exception[3]=(uint8_t)x;
		MB_exception[4]=(uint8_t)(x>>8);
		USART_SEND((uint8_t*)&MB_exception,5, port);
	}

	return (uint32_t)Mb_Error;
}

void fLoadCOM_Param(stCOM_Cfg *pSt_cfg)
{
	memcpy(&stCOMcfg,pSt_cfg,2*sizeof(stCOM_Cfg));
}

stCOM_Cfg* fGet_COM_Param(void)
{
	return (stCOM_Cfg*)&stCOMcfg;

}

uint32_t fGetLastCOM_tm(uint32_t port)
{
	if(port < 2) return stCom[port].lastCom_tm;
	else return 0;
}

uint32_t fGetErrCOM_Cnt(uint32_t port)
{
	if(port < 2) return stCom[port].Error_cnt;
	else return 0;
}

void fClr_LastCOM(uint32_t port)
{
	if(port < 2) stCom[port].lastCom_tm=GetTickCount();
}

//	Main Modbus function process, must be called periodically

uint32_t fMbus_Engine(void)

{
	uint32_t status = NO_ERROR;

	if(stCom[0].RX_bytes != 0 && TimeDiff(stCom[0].lastCom_tm, GetTickCount()) > stCOMcfg.Timeout)
	{

		if(stCom[0].RX_bytes > 7)
		{
			status = Compute_MBUSRequest((uint8_t *)&stCom[0].Buffer, 0, stCOMcfg.Mbus_sID);

			if(status !=SLAVE_NOT_ME)
			{
				if(status != NO_ERROR && status != MEMORY_PARITY_ERROR)
				{
					stCom[0].LastError = status;
					stCom[0].Error_cnt++;
				}
			}

		}
		stCom[0].RX_bytes = 0;
		stCom[0].bRX_BufferFull=false;
		stCom[0].bOverrun=false;
	}

	return status;
}

void TestUart(void)
{
	if(SendEnable)
		USART_SEND(Test_String,sizeof(Test),0);
}


