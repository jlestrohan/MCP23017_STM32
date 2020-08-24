/*
 * mcp23017.h
 *
 *  Created on: 18 apr. 2018
 *      Author: Jack Lestrohan
 */

#ifndef MCP23017_H_
#define MCP23017_H_

#include "stm32g4xx_hal.h"

// Addresses (A0-A2)
#define MCP23017_ADD_20			0x20
#define MCP23017_ADD_21			0x21
#define MCP23017_ADD_22			0x22
#define MCP23017_ADD_23			0x23
#define MCP23017_ADD_24			0x24
#define MCP23017_ADD_25			0x25
#define MCP23017_ADD_26			0x26
#define MCP23017_ADD_27			0x27


/*INPUTS/OUTPUTS */
#define MCP23017_GPA0_Pin		0x00
#define MCP23017_GPA1_Pin		0x01
#define MCP23017_GPA2_Pin		0x02
#define MCP23017_GPA3_Pin		0x03
#define MCP23017_GPA4_Pin		0x04
#define MCP23017_GPA5_Pin		0x05
#define MCP23017_GPA6_Pin		0x06
#define MCP23017_GPA7_Pin		0x07
#define MCP23017_GPB0_Pin		0x08
#define MCP23017_GPB1_Pin		0x09
#define MCP23017_GPB2_Pin		0x0A
#define MCP23017_GPB3_Pin		0x0B
#define MCP23017_GPB4_Pin		0x0C
#define MCP23017_GPB5_Pin		0x0D
#define MCP23017_GPB6_Pin		0x0E
#define MCP23017_GPB7_Pin		0x0F

typedef enum{
	MCP23017_PIN_MODE_INPUT,
	MCP23017_PIN_MODE_OUTPUT
} MCP23017_PinModeIO;

typedef enum{
	MCP23017_PIN_STATE_LOW,
	MCP23017_PIN_STATE_HIGH
} MCP23017_PinModeState;

typedef enum {
	MCP23017_IRQ_MODE_LOW, 			/* to trigger the interrupt whenever the pin is low */
	MCP23017_IRQ_MODE_CHANGE, 		/* to trigger the interrupt whenever the pin changes value */
	MCP23017_IRQ_MODE_RISING,		/* to trigger when the pin goes from low to high */
	MCP23017_IRQ_MODE_FALLING,		/* for when the pin goes from high to low. */
	MCP23017_IRQ_MODE_HIGH 			/* to trigger the interrupt whenever the pin is high. */
} MCP23017_InterruptModeState;

typedef struct {
	I2C_HandleTypeDef*	hi2c;
	uint16_t			addr;
	//uint8_t			gpio[2];
} MCP23017_HandleTypeDef;



//Functions prototypes for hardware abstraction
/**
 * main initialization routine
 * @param hdev
 * @param hi2c
 * @param addr
 */
HAL_StatusTypeDef mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr);

/**
 * Sets the pin mode to either INPUT or OUTPUT
 * @param hdev
 * @param p
 * @param pinMode MCP23017_PinModeIO
 */
HAL_StatusTypeDef mcp23017_pinMode(MCP23017_HandleTypeDef *hdev, uint8_t p, MCP23017_PinModeIO pinMode);

/**
 * Enable a pullUp resistor for a given pin
 * @param hdev
 * @param p
 * @param d
 */
HAL_StatusTypeDef mcp23017_pullUp(MCP23017_HandleTypeDef *hdev, uint8_t p, MCP23017_PinModeState pinPullUpState);

/**
 * Reads a given pin
 * @param hdev
 * @param pin
 * @return
 */
HAL_StatusTypeDef mcp23017_digitalRead(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t *data);

/**
 * Writes to a pin on the MCP23017
 * @param hdev
 * @param pin
 * @param d
 */
HAL_StatusTypeDef mcp23017_digitalWrite(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t d);

/**
 * Sets pinmode (input/output) for all pins once for all
 * @param dev	MCP23017_HandleTypeDef struct
 * @param mode	pinModeState HIGH/LOW
 */
void mcp23017_pinModeAllPins(MCP23017_HandleTypeDef *hdev, MCP23017_PinModeIO pinmode);


/**
 * Sets pull-up state (high/low) for all pins once for all
 * @param hdev
 * @param puState
 */
void mcp23017_pullUpAllPins(MCP23017_HandleTypeDef *hdev, MCP23017_PinModeState puState);


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
HAL_StatusTypeDef mcp23017_setupInterruptPin(MCP23017_HandleTypeDef *hdev, uint8_t pin, uint8_t mode);

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
HAL_StatusTypeDef mcp23017_setupInterrupts(MCP23017_HandleTypeDef *hdev, uint8_t mirroring, uint8_t openDrain, uint8_t polarity);

/**
 * Gets the last interrupt pin
 * @param hdev
 * @param data pointer to the target variable that will contain the result
 * @return HAL_StatusTypeDef I2C mcp23017 instance struct
 */
HAL_StatusTypeDef mcp23017_getLastInterruptPin(MCP23017_HandleTypeDef *hdev, uint8_t *data);

#endif /* MCP23017_H_ */
