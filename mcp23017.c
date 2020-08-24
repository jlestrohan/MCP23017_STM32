/*
 * mcp23017.c
 *
 *  Created on: 18 apr. 2018
 *      Author: Jack Lestrohan
 */

#include "mcp23017.h"
#include "stm32g4xx_hal.h"

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

// Registers
#define MCP23017_IODIRA 		0x00
#define MCP23017_IPOLA 			0x02
#define MCP23017_GPINTENA 		0x04
#define MCP23017_DEFVALA 		0x06
#define MCP23017_INTCONA 		0x08
#define MCP23017_IOCONA 		0x0A
#define MCP23017_GPPUA 			0x0C
#define MCP23017_INTFA 			0x0E
#define MCP23017_INTCAPA 		0x10
#define MCP23017_GPIOA 			0x12
#define MCP23017_OLATA 			0x14

#define MCP23017_IODIRB 		0x01
#define MCP23017_IPOLB 			0x03
#define MCP23017_GPINTENB 		0x05
#define MCP23017_DEFVALB 		0x07
#define MCP23017_INTCONB 		0x09
#define MCP23017_IOCONB 		0x0B
#define MCP23017_GPPUB 			0x0D
#define MCP23017_INTFB 			0x0F
#define MCP23017_INTCAPB 		0x11
#define MCP23017_GPIOB 			0x13
#define MCP23017_OLATB 			0x15

#define MCP23017_INT_ERR 		0xFF

/* functions definitions */
//Functions prototypes for communication
static HAL_StatusTypeDef mcp23017_writeRegister(MCP23017_HandleTypeDef *hdev, uint8_t regAddr, uint8_t regValue);
static HAL_StatusTypeDef mcp23017_updateRegisterBit(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) ;
static HAL_StatusTypeDef mcp23017_readRegister(MCP23017_HandleTypeDef *hdev, uint8_t addr, uint8_t *data);

/**
 * Bit number associated to a given pin
 * @param pin
 * @return
 */
uint8_t bitForPin(uint8_t pin){
	return pin % 8;
}

/**
 * Register address, port dependent, for a given PIN
 * @param pin
 * @param portAaddr
 * @param portBaddr
 * @return
 */
uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return(pin<8) ? portAaddr : portBaddr;
}

/**
 * Writes a value to the given register
 * @param hdev MCP23017_HandleTypeDef struct to the aimed interface
 * @param regAddr Register Address
 * @param regValue Value to write to
 * @return
 */
HAL_StatusTypeDef mcp23017_writeRegister(MCP23017_HandleTypeDef *hdev, uint8_t regAddr, uint8_t regValue) {
	return (HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, regAddr, 1, (uint8_t*) &regValue, 1, 350));
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
	uint8_t regAddr=regForPin(pin,portAaddr,portBaddr);
	uint8_t bit=bitForPin(pin);
	mcp23017_readRegister(hdev, regAddr, &regValue);

	// set the value for the particular bit
	bitWrite(regValue,bit,pValue);

	return (mcp23017_writeRegister(hdev, regAddr,regValue));
}


/**
 * Reads a given register
 * @param hdev
 * @param addr
 * @return
 */
HAL_StatusTypeDef mcp23017_readRegister(MCP23017_HandleTypeDef *hdev, uint8_t addr, uint8_t *data) {
	// read the current GPINTEN
	return (HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, addr, 1, data, 1, 350));
}

/**
 * main initialization routine
 * @param hdev
 * @param hi2c
 * @param addr
 */
HAL_StatusTypeDef mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;

	// all inputs on port A and B
	if (mcp23017_writeRegister(hdev, MCP23017_IODIRA,0xff) == HAL_OK)
		return (mcp23017_writeRegister(hdev, MCP23017_IODIRB,0xff));
	return HAL_ERROR;

}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 * @param hdev
 * @param p
 * @param d
 */
HAL_StatusTypeDef mcp23017_pinMode(MCP23017_HandleTypeDef *hdev, uint8_t p, MCP23017_PinModeIO pinMode) {
	return (mcp23017_updateRegisterBit(hdev, p,(pinMode==MCP23017_PIN_MODE_INPUT),MCP23017_IODIRA,MCP23017_IODIRB));
}

/**
 * Enable a pullUp resistor for a given pin
 * @param hdev
 * @param p
 * @param d
 */
HAL_StatusTypeDef mcp23017_pullUp(MCP23017_HandleTypeDef *hdev, uint8_t p, MCP23017_PinModeState pinPullUpState) {
	return (mcp23017_updateRegisterBit(hdev, p,pinPullUpState,MCP23017_GPPUA,MCP23017_GPPUB));
}

/**
 * Reads a given pin
 * @param hdev
 * @param pin
 * @return
 */
HAL_StatusTypeDef mcp23017_digitalRead(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t *data) {
	uint8_t bit=bitForPin(pin);
	uint8_t regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	HAL_StatusTypeDef ret = mcp23017_readRegister(hdev, regAddr, data);
	*data = (*data >> bit) & 0x1;
	return ret;
}


/**
 * Writes to a pin on the MCP23017
 * @param hdev
 * @param pin
 * @param d
 */
HAL_StatusTypeDef mcp23017_digitalWrite(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t d)
{
	uint8_t gpio;
	uint8_t bit = bitForPin(pin);
	// read the current GPIO output latches
	uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
	if (mcp23017_readRegister(hdev, regAddr, &gpio) != HAL_OK) return HAL_ERROR;
	// set the pin and direction
	bitWrite(gpio, bit, d);

	// write the new GPIO
	regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
	if (mcp23017_writeRegister(hdev, regAddr, gpio) != HAL_OK) return HAL_ERROR;

	return HAL_OK;
}

/**
 * Sets pinmode (input/output) for all pins once for all
 * @param dev	MCP23017_HandleTypeDef struct
 * @param mode	pinModeState HIGH/LOW
 */
void mcp23017_pinModeAllPins(MCP23017_HandleTypeDef *hdev, MCP23017_PinModeIO pinmode)
{
	for(uint8_t i=0; i<15; i++){
		mcp23017_pinMode(hdev, i, pinmode);
	}
}

/**
 * Sets pull-up state (high/low) for all pins once for all
 * @param hdev
 * @param puState
 */
void mcp23017_pullUpAllPins(MCP23017_HandleTypeDef *hdev, MCP23017_PinModeState puState)
{
	for(uint8_t i=0; i<15; i++){
		mcp23017_pullUp(hdev, i, puState);
	}
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
HAL_StatusTypeDef mcp23017_setupInterruptPin(MCP23017_HandleTypeDef *hdev, uint8_t pin, MCP23017_InterruptModeState mode)
{
	/* set the pin interrupt control (0 means change, 1 means compare against
	   given value); */
	if (!mcp23017_updateRegisterBit(hdev, pin, (mode != MCP23017_IRQ_MODE_CHANGE), MCP23017_INTCONA, MCP23017_INTCONB) != HAL_OK) return HAL_ERROR;
	/* if the mode is not CHANGE, we need to set up a default value, different
	   value triggers interrupt */

	/* In a RISING interrupt the default value is 0, interrupt is triggered when
	 the pin goes to 1. In a FALLING interrupt the default value is 1, interrupt
	 is triggered when pin goes to 0. */
	if (!mcp23017_updateRegisterBit(hdev, pin, (mode == MCP23017_IRQ_MODE_FALLING), MCP23017_DEFVALA, MCP23017_DEFVALB) != HAL_OK) return HAL_ERROR;

	/* enable the pin for interrupt */
	if (!mcp23017_updateRegisterBit(hdev, pin, mode == MCP23017_IRQ_MODE_CHANGE, MCP23017_GPINTENA, MCP23017_GPINTENB) != HAL_OK) return HAL_ERROR;

	return HAL_OK;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same
 * configuration.
 * @param mirroring Mirroring will OR both INTA and INTB pins.
 * @param openDrain Opendrain will set the INT pin to value or open drain.
 * @param polarity polarity will set LOW or HIGH on interrupt.
 * @return HAL_StatusTypeDef error handling
 *
 * Default values after Power On Reset are: (false, false, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the
 * interupt handling as FALLING with the default configuration.
 */
HAL_StatusTypeDef mcp23017_setupInterrupts(MCP23017_HandleTypeDef *hdev, uint8_t mirroring, uint8_t openDrain, uint8_t polarity) {
	// configure the port A
	uint8_t ioconfValue;
	if (mcp23017_readRegister(hdev, MCP23017_IOCONA, &ioconfValue) != HAL_OK) return HAL_ERROR;
	bitWrite(ioconfValue, 6, mirroring); /* mirroring here is recommended by default */
	bitWrite(ioconfValue, 2, openDrain);
	bitWrite(ioconfValue, 1, polarity);
	if (mcp23017_writeRegister(hdev, MCP23017_IOCONA, ioconfValue) != HAL_OK) return HAL_ERROR;

	// Configure the port B
	if (mcp23017_readRegister(hdev, MCP23017_IOCONB, &ioconfValue) != HAL_OK) return HAL_ERROR;
	bitWrite(ioconfValue, 6, mirroring);
	bitWrite(ioconfValue, 2, openDrain);
	bitWrite(ioconfValue, 1, polarity);
	if (mcp23017_writeRegister(hdev, MCP23017_IOCONB, ioconfValue) != HAL_OK) return HAL_ERROR;

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
	// try port A
	uint8_t intf;
	if (mcp23017_readRegister(hdev, MCP23017_INTFA, &intf) != HAL_OK) return HAL_ERROR;
	for (int i = 0; i < 8; i++)
		if (bitRead(intf, i))
			*data = i;

	// try port B
	if (mcp23017_readRegister(hdev, MCP23017_INTFB, &intf) != HAL_OK) return HAL_ERROR;
	for (int i = 0; i < 8; i++)
		if (bitRead(intf, i))
			*data = i + 8;

	return HAL_OK;
}

/**
 * Gets the value of the last interrupt pin
 * @param hdev
 * @param data target var passed by reference will contain the value of the last interrupt pin
 * @return
 */
HAL_StatusTypeDef mcp23017_getLastInterruptPinValue(MCP23017_HandleTypeDef *hdev, uint8_t *data)
{
	uint8_t intPin;
	if (mcp23017_getLastInterruptPin(hdev, &intPin) != HAL_OK) return HAL_ERROR;
	if (intPin != MCP23017_INT_ERR) {
		uint8_t intcapreg = regForPin(intPin, MCP23017_INTCAPA, MCP23017_INTCAPB);
		uint8_t bit = bitForPin(intPin);
		if(mcp23017_readRegister(hdev, intcapreg, data) != HAL_OK) return HAL_ERROR;
		*data = (*data >> bit) & (0x01);
	}

	return HAL_OK;
}

