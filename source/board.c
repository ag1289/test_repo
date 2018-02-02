/* This is a template for board specific configuration created by MCUXpresso Project Generator. Enjoy! */

#include <stdint.h>
#include "MK22F12810.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/*!
 * @brief initialize debug console to enable printf for this demo/example */



void BOARD_Init(void) {

	/* pin output configuration */
	gpio_pin_config_t gpio_config =
	{
		kGPIO_DigitalOutput,
	    1,
	};

	port_digital_filter_config_t input_digitalFilter = {.clockSource = kPORT_LpoClock, .digitalFilterWidth = 100};


	GPIO_PinInit(AXIS_OUT1_GPIO, AXIS_OUT1_GPIO_PIN, &gpio_config);
	GPIO_PinInit(AXIS_OUT2_GPIO, AXIS_OUT2_GPIO_PIN, &gpio_config);
	GPIO_PinInit(INH_BRIDGE1_GPIO, INH_BRIDGE1_GPIO_PIN, &gpio_config);
	GPIO_PinInit(RFM_RST_GPIO, RFM_RST_GPIO_PIN, &gpio_config);
	GPIO_PinInit(KEYB7_GPIO, KEYB7_GPIO_PIN, &gpio_config);
	GPIO_PinInit(STATUSLED_GPIO, STATUSLED_GPIO_PIN, &gpio_config);


	// GPIO_PinInit(RFM_RST_GPIO, RFM_RST_GPIO_PIN, &gpio_config);  PCF2127 uses this pin to your own watchdog output RST

	/* Set init pin values */

	GPIO_WritePinOutput(INH_BRIDGE1_GPIO, INH_BRIDGE1_GPIO_PIN, 0);
	GPIO_WritePinOutput(AXIS_OUT1_GPIO, AXIS_OUT1_GPIO_PIN,0);
    GPIO_WritePinOutput(AXIS_OUT2_GPIO, AXIS_OUT2_GPIO_PIN,0);
    GPIO_WritePinOutput(KEYB7_GPIO, KEYB7_GPIO_PIN,0);
	GPIO_WritePinOutput(RFM_RST_GPIO, RFM_RST_GPIO_PIN, 1);
	GPIO_WritePinOutput(STATUSLED_GPIO, STATUSLED_GPIO_PIN, 1);

	// Digital filter for KEYB inputs

	PORT_SetDigitalFilterConfig(PORTD, &input_digitalFilter);
	PORT_EnablePinsDigitalFilter(PORTD, 0x70, true);
/*	PORT_SetDigitalFilterConfig(PORTB, input_digitalFilter);
	PORT_EnablePinsDigitalFilter(PORTB, 0xf0000, true);*/
}


