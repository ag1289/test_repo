
#ifndef LoRa_SPI_h
#define LoRa_SPI_h

#include<stdint.h>
#include<stdbool.h>

void vSpiInit(uint32_t); 		/** initialize hardware SPI config, SPI_CLK = Fcpu/4 **/
void vSpiWrite(uint16_t ); 	/** SPI send one word **/
uint8_t bSpiRead(uint8_t ); /** SPI read one byte **/
void vSpiBurstWrite(uint8_t , uint8_t *, uint8_t ); 	/** SPI burst send N byte **/
void vSpiBurstRead(uint8_t , uint8_t *, uint8_t );		/** SPI burst rend N byte **/

#endif
