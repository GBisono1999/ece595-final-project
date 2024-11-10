/**
 * @file pwm_driver.h
 * @brief Header file for the Adafruit PCA9685 PWN LED controller.
 *
 * This file contains the function defintions for the Adafruit driver.
 *
 * It uses the i2c driver to access the registers of the Adafruit PWM LED controller.
 *
 * The PCA9685 has the following pinout:
 *  - PCA9685 Pin 14 (VSS)(GND)      <--> MSP432 LaunchPad GND
 *  - PCA9685 Pin 23 (~OE)           <--> NOT SUPPORTED (YET)
 *  - PCA9685 Pin 26 (SCL)           <--> MSP432 LaunchPad Pin P6.5
 *  - PCA9685 Pin 27 (SDA)           <--> MSP432 LaunchPad Pin P6.4
 *  - PCA9685 Pin 28 (VDD)(VCC)      <--> MSP432 LaunchPad Pin P4.5 //check out the OPT3001.h/.c file for more details.
 *  - PCA9685 (V+)                   <--> DO NOT CONNECT TO BOARD
 *
 *  @note For PCA9685 Pin 28, do not connect it directly to the board.
 *  @note For PCA9685 V+, use external power supply as this will power the pins.
 *
 *
 * @author Geovanni Bisono
 *
 */


#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>
#include <msp.h>
#include "i2c.h"
#include "../../inc/Clock.h"

//#defines below taken from adruino library of PCA9685
// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

void PCA9685_Init();

//grabbed from the arduino library
//void PCA9685_setPWMFreq(float freq);

//grabbed from the arduino library
//void PCA9685_setPWM(uint8_t num, uint16_t on, uint16_t off);

//grabbed from the arduino library
//void PCA9685_setOscillatorFrequency(uint32_t freq);

//grabbed from the arduino library
uint8_t PCA9685_Read_Register(uint8_t register_address);

//grabbed from the arduino library
void PCA9685_Write_Register(uint8_t register_address, uint8_t register_data);

#endif //PWM_DRIVER_H
