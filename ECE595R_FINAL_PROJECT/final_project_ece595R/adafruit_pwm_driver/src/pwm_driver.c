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
//This is your pwm.begin() basically, or should be at least.
void PCA9685_Init()
{

    EUSCI_B1_I2C_Init();
    //Configure P4.5 to GPIO
    P4->SEL0 &= ~0x20;
    P4->SEL1 &= ~0x20;

    //P4.5 as output
    P4->DIR |= 0x20;

    //Drive P4.5 high as it is supplying power
    P4->OUT |= 0x20;

    //Add a small delay for the pins to initialize
    Clock_Delay1ms(1);

    //set a default freq. EXTCLK not support yet.
    PCA9685_setPWMFreq(1000);
}

//void EUSCI_B1_I2C_Send_Data(uint8_t slave_address, uint8_t *data_buffer)

//void EUSCI_B1_I2C_Receive_Data(uint8_t slave_address, uint8_t *data_buffer, uint16_t packet_length)

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

    uint8_t oldmode[1];

    EUSCI_B1_I2C_Receive_Data(PCA9685_I2C_ADDRESS, oldmode);
    uint8_t newmode = (uint8_t)(oldmode[0] & ~MODE1_RESTART) | MODE1_SLEEP; //Set to sleep

    //[Start][Slave Address][R/~W][A][Control Register][A][Data][A][P]

    //uint8_t setMode1[] = {PCA9685_MODE1, newmode};
    //void EUSCI_B1_I2C_Send_Data(uint8_t slave_address, uint8_t *data_buffer)

    uint8_t data[2];

    data[0] = PCA9685_MODE1;
    data[1] = newmode;
    EUSCI_B1_I2C_Send_Data(PCA9685_I2C_ADDRESS, data);

    data[0] = PCA9685_PRESCALE;
    data[1] = prescale;
    EUSCI_B1_I2C_Send_Data(PCA9685_I2C_ADDRESS, data);

    data[0] = PCA9685_MODE1;
    data[1] = oldmode[0];
    EUSCI_B1_I2C_Send_Data(PCA9685_I2C_ADDRESS, data);

    //EUSCI_B1_I2C_Send_A_Byte(PCA9685_MODE1, newmode);
    //EUSCI_B1_I2C_Send_A_Byte(PCA9685_PRESCALE, prescale);
    //EUSCI_B1_I2C_Send_A_Byte(PCA9685_MODE1, oldmode[0]);

    Clock_Delay1ms(5);

    data[0] = PCA9685_MODE1;
    data[1] = (oldmode[0] | MODE1_RESTART | MODE1_AI);
    EUSCI_B1_I2C_Send_Data(PCA9685_I2C_ADDRESS, data);

    //EUSCI_B1_I2C_Send_A_Byte(PCA9685_MODE1, (oldmode[0] | MODE1_RESTART | MODE1_AI));
}


/*
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 *  @return 0 if successful, otherwise 1
 */
void PCA9685_setPWM(uint8_t num, uint16_t on, uint16_t off)
{
    uint8_t buffer[5];
    buffer[0] = PCA9685_LED0_ON_L + 4 * num;
    buffer[1] = on;
    buffer[2] = on >> 8;
    buffer[3] = off;
    buffer[4] = off >> 8;

    EUSCI_B1_I2C_Send_Data(PCA9685_I2C_ADDRESS, buffer);
}
