/**
 * --------------------------------------------------------------------------
 * mcp23017.h
 *
 *  Created on: Aug 16, 2020
 *      Author: Jack Lestrohan (c) Cobalt Audio - 2020
 * --------------------------------------------------------------------------
 */
#include "mcp23017.h"
#include "cmsis_os2.h"
#include "stm32g4xx_hal.h"

/* max delay given to all I2C stuff to initialize itself */
#define I2C_READYNESS_DELAY		500

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

/**
 * Registers addresses.
 * The library use addresses for IOCON.BANK = 0.
 * See "3.2.1 Byte mode and Sequential mode".
 */
#define MCP23017_IODIRA 		0x00	///< Controls the direction of the data I/O for port A.
#define MCP23017_IPOLA 			0x02	///< Configures the polarity on the corresponding GPIO_ port bits for port A.
#define MCP23017_GPINTENA 		0x04	///< Controls the interrupt-on-change for each pin of port A.
#define MCP23017_DEFVALA 		0x06	///< Controls the default comparaison value for interrupt-on-change for port A.
#define MCP23017_INTCONA 		0x08	///< Controls how the associated pin value is compared for the interrupt-on-change for port A.
#define MCP23017_IOCONA 		0x0A	///< Controls the device.
#define MCP23017_GPPUA 			0x0C	///< Controls the pull-up resistors for the port A pins.
#define MCP23017_INTFA 			0x0E	///< Reflects the interrupt condition on the port A pins.
#define MCP23017_INTCAPA 		0x10	///< Captures the port A value at the time the interrupt occured.
#define MCP23017_GPIOA 			0x12	///< Reflects the value on the port A.
#define MCP23017_OLATA 			0x14	///< Provides access to the port A output latches.

#define MCP23017_IODIRB 		0x01	///< Controls the direction of the data I/O for port B.
#define MCP23017_IPOLB 			0x03	///< Configures the polarity on the corresponding GPIO_ port bits for port B.
#define MCP23017_GPINTENB 		0x05	///< Controls the interrupt-on-change for each pin of port B.
#define MCP23017_DEFVALB 		0x07	///< Controls the default comparaison value for interrupt-on-change for port B.
#define MCP23017_INTCONB 		0x09	///< Controls how the associated pin value is compared for the interrupt-on-change for port B.
#define MCP23017_IOCONB 		0x0B	///< Controls the device.
#define MCP23017_GPPUB 			0x0D	///< Controls the pull-up resistors for the port B pins.
#define MCP23017_INTFB 			0x0F	///< Reflects the interrupt condition on the port B pins.
#define MCP23017_INTCAPB 		0x11	///< Captures the port B value at the time the interrupt occured.
#define MCP23017_GPIOB 			0x13	///< Reflects the value on the port B.
#define MCP23017_OLATB 			0x15	///< Provides access to the port B output latches.

#define MCP23017_INT_ERR		0xFF

/**
 * Register address, port dependent, for a given PIN
 * @param pin
 * @param portAaddr
 * @param portBaddr
 * @return
 */
static uint8_t mcp23017_regForPin (uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return(pin<8) ? portAaddr : portBaddr;
}

/**
 * Bit number associated to a given pin
 * @param pin
 * @return
 */
static uint8_t mcp23017_bitForPin(uint8_t pin){
	return pin % 8;
}

/**
 * Reads a given register
 * @param hdev
 * @param addr
 * @return
 */
HAL_StatusTypeDef mcp23017_readRegister(MCP23017_HandleTypeDef *hdev, uint8_t addr, uint8_t *data) {
	// read the current GPINTEN
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, addr, 1, data, 1, HAL_MAX_DELAY);
	return ret;
}


/**
 * Writes a value to the given register
 * @param hdev MCP23017_HandleTypeDef struct to the aimed interface
 * @param regAddr Register Address
 * @param regValue Value to write to
 * @return
 */
static HAL_StatusTypeDef mcp23017_writeRegister(MCP23017_HandleTypeDef *hdev, uint8_t regAddr, uint8_t regValue)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, regAddr, 1, (uint8_t*) &regValue, 1, HAL_MAX_DELAY);
	return ret;
}

/**
 * Helper to update a single bit of an A/B register.
 * 		Reads the current register value
 * 		Writes the new register value
 * @param hdev
 * @param pin
 * @param pValue
 * @param portAaddr
 * @param portBaddr
 */
HAL_StatusTypeDef mcp23017_updateRegisterBit(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr = mcp23017_regForPin(pin,portAaddr,portBaddr);
	uint8_t bit = mcp23017_bitForPin(pin);
	mcp23017_readRegister(hdev, regAddr, &regValue);

	// set the value for the particular bit
	bitWrite(regValue,bit,pValue);

	return (mcp23017_writeRegister(hdev, regAddr,regValue));
}

/**
 * main initialization routine
 * @param hdev
 * @param hi2c
 * @param addr
 */
HAL_StatusTypeDef mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
	HAL_StatusTypeDef ret;

	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;

	ret = HAL_I2C_IsDeviceReady(hi2c, hdev->addr, 20, I2C_READYNESS_DELAY);
	if (ret != HAL_OK) return ret;

	//BANK = 	0 : sequential register addresses
	//MIRROR = 	0 : use configureInterrupt
	//SEQOP = 	1 : sequential operation disabled, address pointer does not increment
	//DISSLW = 	0 : slew rate enabled
	//HAEN = 	0 : hardware address pin is always enabled on 23017
	//ODR = 	0 : open drain output
	//INTPOL = 	0 : interrupt active low
	// bit0 - unplemented read as 0
	uint8_t byte = 0 | 1 << 5 | 1 << 6;

	for (uint8_t i = 0; i < 2; i++) {
		ret = mcp23017_writeRegister(hdev, MCP23017_IOCONA+i, byte);
		if (ret != HAL_OK) return ret;
	}

	return HAL_OK;
}

/**
 *
 * @param hdev
 * @param port
 * @param directions
 * @param pullups
 * @param inverted
 * @return
 */
HAL_StatusTypeDef mcp23017_portMode(MCP23017_HandleTypeDef *hdev, MCP23017Port_t port, MCP23017_PinModeIO_t pinmode, MCP23017_PinPolarity_t pinpolarity)
{
	HAL_StatusTypeDef ret;

	ret = mcp23017_writeRegister(hdev, MCP23017_IODIRA + port,
			(pinmode == MCP23017_PIN_MODE_INPUT || pinmode == MCP23017_PIN_MODE_INPUT_PULLUP) ? 0xFF : 0x00);
	if (ret != HAL_OK) return ret;

	ret = mcp23017_writeRegister(hdev, MCP23017_GPPUA + port, (pinmode == MCP23017_PIN_MODE_INPUT_PULLUP ? 0xff : 0x00));
	if (ret != HAL_OK) return ret;

	ret = mcp23017_writeRegister(hdev, MCP23017_IPOLA + port, pinpolarity ? 0xff : 0x00);
	if (ret != HAL_OK) return ret;

	return HAL_OK;
}

/**
 *
 * @param hdev
 * @param pin
 * @param mode
 * @param inverted
 * @return
 */
HAL_StatusTypeDef mcp23017_pinMode(MCP23017_HandleTypeDef *hdev, uint8_t pin, MCP23017_PinModeIO_t mode, MCP23017_PinPolarity_t polarity)
{
	HAL_StatusTypeDef ret;

	/* if input we set IODIRA or IODIRB (depending on pin number) to 1 */
	ret = mcp23017_updateRegisterBit(hdev, pin, (mode == MCP23017_PIN_MODE_INPUT || mode == MCP23017_PIN_MODE_INPUT_PULLUP),
			MCP23017_IODIRA, MCP23017_IODIRB);
	if (ret != HAL_OK) return ret;

	/* we also need to control the polarity, will be ignored anyway if output, 1 if pol inverted */
	ret = mcp23017_updateRegisterBit(hdev, pin, polarity == MCP23017_PIN_POLARITY_INVERTED, MCP23017_IPOLA, MCP23017_IPOLB);
	if (ret != HAL_OK) return ret;

	/* then we need to setup the pull up for that pin if MCP23017_PIN_MODE_INPUT_PULLUP was selected */
	ret = mcp23017_updateRegisterBit(hdev, pin, mode == MCP23017_PIN_MODE_INPUT_PULLUP, MCP23017_GPPUA, MCP23017_GPPUB);
	if (ret != HAL_OK) return ret;

	return HAL_OK;
}

/**
 * Reads a given pin
 * @param hdev
 * @param pin
 * @return
 */
HAL_StatusTypeDef mcp23017_digitalRead(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t *data) {
	uint8_t bit = mcp23017_bitForPin(pin);
	uint8_t regAddr = mcp23017_regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
	HAL_StatusTypeDef ret = mcp23017_readRegister(hdev, regAddr, data);
	*data = (*data >> bit) & 0x1;
	return ret;
}

/**
 * Writes to a given pin
 * @param hdev
 * @param pin
 * @param data
 * @return
 */
HAL_StatusTypeDef mcp23017_digitalWrite(MCP23017_HandleTypeDef *hdev, uint8_t pin, GPIO_PinState pinState)
{
	uint8_t data;
	HAL_StatusTypeDef ret;
	uint8_t bit = mcp23017_bitForPin(pin);

	// read the current GPIO output latches
	uint8_t regAddr = mcp23017_regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
	ret = mcp23017_readRegister(hdev, regAddr, &data);
	if (ret != HAL_OK) return ret;

	// set the pin and direction
	bitWrite(data, bit, pinState);

	// write the new GPIO
	regAddr = mcp23017_regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
	return (mcp23017_writeRegister(hdev, regAddr, data));
}

/**
 *
 * @param hdev
 * @param mode
 * @return
 */
HAL_StatusTypeDef mcp23017_setInterruptMode(MCP23017_HandleTypeDef *hdev, MCP23017_InterruptMode_t mode)
{
	/* we need to set or unset bit 6 of IOCON A or IOCON B */

	uint8_t data;
	for (uint8_t i = 0; i < 2; i++) {
		/* first we read what's in both IOCON regs, we set or unset the bit and we write it back */
		mcp23017_readRegister(hdev, MCP23017_IOCONA + i, &data);
		if (mode) data |= 1 << 6;
		else data &= ~(1 << 6);
		mcp23017_writeRegister(hdev, MCP23017_IOCONA+i, data);
	}
	return HAL_OK;
}

/**
 * Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information
 * about the port / value that caused the interrupt or you read the port itself.
 * Check the datasheet can be confusing.
 * @param hdev
 * @param pin Pin to set
 * @param mode Mode to set the pin to
 */
HAL_StatusTypeDef mcp23017_setupInterruptPin(MCP23017_HandleTypeDef *hdev, uint8_t pin, MCP23017_InterruptModeState_t mode)
{
	HAL_StatusTypeDef ret;

	// MCP23017_IRQ_MODE_CHANGE, 		/* to trigger the interrupt whenever the pin changes value */
	// MCP23017_IRQ_MODE_RISING,		/* to trigger when the pin goes from low to high */
	// MCP23017_IRQ_MODE_FALLING,		/* for when the pin goes from high to low. */

	/* In a RISING interrupt the default value is 0, interrupt is triggered when
		 the pin goes to 1. In a FALLING interrupt the default value is 1, interrupt
		 is triggered when pin goes to 0. */
	/* we set DEFVAL to 1 to be compared to 0 when the pin will be falling;. or not */
	ret = mcp23017_updateRegisterBit(hdev, pin, (mode == MCP23017_IRQ_MODE_FALLING), MCP23017_DEFVALA, MCP23017_DEFVALB);
	if (ret != HAL_OK) return HAL_ERROR;

	/* set the pin interrupt control (0 means change, 1 means compare against given value); */
	ret = mcp23017_updateRegisterBit(hdev, pin, (mode != MCP23017_IRQ_MODE_CHANGE), MCP23017_INTCONA, MCP23017_INTCONB);
	if (ret != HAL_OK) return HAL_ERROR;

	/* enable the pin for interrupt */
	ret = mcp23017_updateRegisterBit(hdev, pin, mode == MCP23017_IRQ_MODE_CHANGE, MCP23017_GPINTENA, MCP23017_GPINTENB);
	if (ret != HAL_OK) return HAL_ERROR;

	return HAL_OK;
}

/**
 * Gets the last interrupt pin
 * @param hdev
 * @param data pointer to the target variable that will contain the result
 * @return HAL_StatusTypeDef I2C mcp23017 instance struct
 */
HAL_StatusTypeDef mcp23017_getLastInterruptPin(MCP23017_HandleTypeDef *hdev, uint8_t *data)
{
	HAL_StatusTypeDef ret;
	uint8_t intf;
	for (uint8_t j = 0; j < 2; j++) {

		ret = mcp23017_readRegister(hdev, MCP23017_INTFA + j, &intf);
		if (ret != HAL_OK) return HAL_ERROR;

		for (int i = 0; i < 8; i++) {
			if (bitRead(intf, i)) {
				*data = i;
				return HAL_OK; /* no need to wait we know which pin is responsible */
			}
		}
	}
	return HAL_OK;
}

/**
 * Gets the value of the last interrupt pin
 * @param hdev
 * @param data target var passed by reference will contain the value of the last interrupt pin
 * @return
 */
HAL_StatusTypeDef mcp23017_getLastInterruptPinValue(MCP23017_HandleTypeDef *hdev, uint16_t *data)
{
	uint8_t intPin;
	HAL_StatusTypeDef ret;
	uint8_t tmpdata = 0;

	/*if (mcp23017_getLastInterruptPin(hdev, &intPin) != HAL_OK) return HAL_ERROR;
	if (intPin != MCP23017_INT_ERR) {
		uint8_t intcapreg = mcp23017_regForPin(intPin, MCP23017_INTCAPA, MCP23017_INTCAPB);
		uint8_t bit = mcp23017_bitForPin(intPin);
		if(mcp23017_readRegister(hdev, intcapreg, data) != HAL_OK) return HAL_ERROR;
		*data = (*data >> bit) & (0x01);
	}*/

	ret = mcp23017_readRegister(hdev, MCP23017_GPIOA, &tmpdata);
	if (ret != HAL_OK) return ret;
	*data |= tmpdata << 8;
	ret = mcp23017_readRegister(hdev, MCP23017_GPIOB, &tmpdata);
		if (ret != HAL_OK) return ret;
	*data |= tmpdata;

	return HAL_OK;
}
