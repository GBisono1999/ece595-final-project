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
 * To write to a specific register, follow the following bus transaction:
 *
 * [Start][Slave Address][R/~W][A][Control Register][A][Data][A][P]
 *
 *  - [Start]            : Start Bit
 *  - [Slave Address]    : The address for the device, default is 0x40h.
 *  - [R/~W]             : Read/Write Bit
 *      - Read           : Logic 1 is selected
 *      - Write          : Logic 0 is selected
 *  - [A]                : Acknowledge from slave
 *  - [Control Register] : Control Register that will be selected.
 *  - [A]                : Acknowledge from slave
 *  - [Data]             : Data for the Control Register that was selected.
 *  - [A]                : Acknowledge from slave
 *  - [P]                : Stop Bit
 *
 * @author Geovanni Bisono
 *
 */

#include "../inc/pwm_driver.h"
#include <stdint.h>


void PCA9685_Write_Register(uint8_t register_address, uint8_t register_data)
{
    uint8_t buffer[] =
        {
             register_address,
             register_data & 0xFF
        };

    EUSCI_B1_I2C_Send_Multiple_Bytes(PCA9685_I2C_ADDRESS, buffer, sizeof(buffer));
}

uint8_t PCA9685_Read_Register(uint8_t register_address)
{
    //printf("A\n");
    EUSCI_B1_I2C_Send_A_Byte(PCA9685_I2C_ADDRESS, register_address);
    //printf("B\n");
    uint8_t received_data = EUSCI_B1_I2C_Receive_A_Byte(PCA9685_I2C_ADDRESS);
    //printf("C\n");
    return received_data;
}

void PCA9685_Init()
{
    // Initialize I2C using EUSCI_B1 module
    printf("EUSCI_B1_I2C_Init Begin\n");
    EUSCI_B1_I2C_Init();
    printf("EUSCI_B1_I2C_Init End\n");
    Clock_Delay1ms(100);

    printf("PCA9685_setPWMFreq Begin\n");
    PCA9685_setPWMFreq(1000.0);
    printf("PCA9685_setPWMFreq End\n");
}

void PCA9685_setPWMFreq(float freq)
{
    if (freq < 1)
        freq = 1;
    if (freq > 3500)
        freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

    float prescaleval = ((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
        prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX)
        prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;


    printf("PCA9685_Read_Register PCA9685_MODE1 Begin\n");
    uint8_t oldmode = PCA9685_Read_Register(PCA9685_MODE1);
    printf("PCA9685_Read_Register PCA9685_MODE1 End\n");

    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; //Set to sleep

    //[Start][Slave Address][R/~W][A][Control Register][A][Data][A][P])

    PCA9685_Write_Register(PCA9685_MODE1, newmode);
    PCA9685_Write_Register(PCA9685_PRESCALE, prescale);
    PCA9685_Write_Register(PCA9685_MODE1, oldmode);

    Clock_Delay1ms(50);

    PCA9685_Write_Register(PCA9685_MODE1, (oldmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

void PCA9685_setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    //uint8_t a1 = (PCA9685_LED0_ON_L + 4 * num);

    //PCA9685_Write_Register(PCA9685_LED0_ON_L, on);

    //uint8_t a2 = (PCA9685_LED0_ON_L + 4 * num) + 1;
    //PCA9685_Write_Register(PCA9685_LED0_ON_H, on >> 8);

    //uint8_t a3 = (PCA9685_LED0_ON_L + 4 * num) + 2;

    //PCA9685_Write_Register(PCA9685_LED0_OFF_L, off);
    //uint8_t a4 = (PCA9685_LED0_ON_L + 4 * num) + 3;

    //PCA9685_Write_Register(PCA9685_LED0_0FF_H, off >> 8);


    uint8_t buffer[5];
    buffer[0] = PCA9685_LED0_ON_L + 4 * num;
    buffer[1] = on & 0xFF;
    buffer[2] = (on >> 8)& 0xFF;
    buffer[3] = off& 0xFF;
    buffer[4] = (off >> 8)& 0xFF;

    EUSCI_B1_I2C_Send_Multiple_Bytes(PCA9685_I2C_ADDRESS, buffer, sizeof(buffer));
    printf("YES\n");
    //uint8_t r1 =

}
