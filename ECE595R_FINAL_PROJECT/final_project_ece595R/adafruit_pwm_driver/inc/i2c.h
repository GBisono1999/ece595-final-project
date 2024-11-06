/**
 * @file i2c.h
 * @brief Header file to setup I2C for the EUSCI_B1_I2C driver.
 *
 * @note This driver is configured to work with the Adafruit I2C driver file.
 *
 * This function assumes that the necessary pin configurations for I2C communication have been performed
 *       on the corresponding pins. The output from the pins will be observed using an oscilloscope.
 *       - P6.4 (SDA)
 *       - P6.5 (SCL)
 * @author Geovanni Bisono
 * @note Used @Aaron Nanas' implementation as a base overall.
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "msp.h"



void EUSCI_B1_I2C_Init();

void EUSCI_B1_I2C_Send_A_Byte(uint8_t slave_address, uint8_t data);

void EUSCI_B1_I2C_Send_Data(uint8_t slave_address, uint8_t *data_buffer);

void EUSCI_B1_I2C_Receive_Data(uint8_t slave_address, uint8_t *data_buffer);



#endif //I2C_H
