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


void PCA9685_Init();

//grabbed from the arduino library
void PCA9685_setPWMFreq(float freq);

//grabbed from the arduino library
void PCA9685_setPWM(uint8_t num, uint16_t on, uint16_t off);

//grabbed from the arduino library
void PCA9685_read();

//grabbed from the arduino library
void PCA9685_write();

#endif //PWM_DRIVER_H
