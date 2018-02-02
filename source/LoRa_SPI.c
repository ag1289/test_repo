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
 * file       HopeDuino_SPI.cpp
 * brief      for HopeRF's EVB to use Hardware SPI
 * hardware   HopeRF's EVB
 *            
 *
 * version    1.1
 * date       Jan 15 2015
 * author     QY Ruan
 */

#include <string.h>
#include "LoRa_SPI.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK22F12810.h"
#include "fsl_gpio.h"
#include "fsl_dspi.h"

#define SPI_LORA	SPI0

dspi_master_handle_t g_m_handle;

uint8_t SPI_Tx_Buffer[128], SPI_Rx_Buffer[128];

/**********************************************************
**Name: 	vSpiInit
**Func: 	Init Spi Config
**Note: 	SpiClk = Fcpu/4
**********************************************************/
void vSpiInit(uint32_t baudrate)
{

	dspi_master_config_t  masterConfig;

	masterConfig.whichCtar                                = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate                      = baudrate;
	masterConfig.ctarConfig.bitsPerFrame                  = 8;
	masterConfig.ctarConfig.cpol                          = kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha                          = kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction                     = kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec        = 1000000000 / baudrate ;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec    = 1000000000 / baudrate ;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000 / baudrate ;
	masterConfig.whichPcs                                 = kDSPI_Pcs0;
	masterConfig.pcsActiveHighOrLow                       = kDSPI_PcsActiveLow;
	masterConfig.enableContinuousSCK                      = false;
	masterConfig.enableRxFifoOverWrite                    = false;
	masterConfig.enableModifiedTimingFormat               = false;
	masterConfig.samplePoint                              = kDSPI_SckToSin0Clock;

	DSPI_MasterInit(SPI_LORA, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
	DSPI_MasterTransferCreateHandle(SPI_LORA, &g_m_handle, NULL, NULL);

}


/**********************************************************
**Name:	 	vSpiWrite
**Func: 	SPI Write One word
**Input: 	Write word
**Output:	none
**********************************************************/
void vSpiWrite(uint16_t data)
{
	uint8_t buffer[2]={(uint8_t)(data>>8), (uint8_t)data };

	vSpiBurstWrite(buffer[0], &buffer[1], 1);

}

/**********************************************************
**Name:	 	bSpiRead
**Func: 	SPI Read One byte
**Input: 	readout addresss
**Output:	readout byte
**********************************************************/
uint8_t bSpiRead(uint8_t addr)
{
	uint8_t tmp;
	vSpiBurstRead(addr, &tmp, 1);
	return tmp;
}

/**********************************************************
**Name:	 	vSpiBurstWirte
**Func: 	burst wirte N byte
**Input: 	array length & start address & head pointer
**Output:	none
**********************************************************/
void vSpiBurstWrite(uint8_t addr, uint8_t *data, uint8_t nbyte)
{
	dspi_transfer_t masterXfer;

	SPI_Tx_Buffer[0] = addr | 0x80;				// First bit must be 1 for write operation
	memcpy(&SPI_Tx_Buffer[1], data, ++nbyte);

	masterXfer.txData      = SPI_Tx_Buffer;
	masterXfer.rxData      = SPI_Rx_Buffer;
	masterXfer.dataSize    = nbyte;
	masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

	DSPI_MasterTransferBlocking(SPI_LORA, &masterXfer);
}

/**********************************************************
**Name:	 	vSpiBurstRead
**Func: 	burst read N uint8_t
**Input: 	array length & start address & head pointer
**Output:	none
**********************************************************/
void vSpiBurstRead(uint8_t addr, uint8_t *data, uint8_t nbyte)
{
	dspi_transfer_t masterXfer;

	if(!nbyte)return;

	SPI_Tx_Buffer[0] = addr & 0x7f;			// First bit must be 0 for read operation
	memset(&SPI_Tx_Buffer[1], 0, ++nbyte);	// Send all zero to read

	masterXfer.txData      = SPI_Tx_Buffer;
	masterXfer.rxData      = SPI_Rx_Buffer;
	masterXfer.dataSize    = nbyte--;
	masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

	DSPI_MasterTransferBlocking(SPI_LORA, &masterXfer);
	memcpy((void *)data, (void *)&SPI_Rx_Buffer[1], nbyte);
}

