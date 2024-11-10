/**
 * @file i2c.c
 * @brief Source code to setup I2C for the EUSCI_B1_I2C driver.
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

#include "../inc/i2c.h"


void EUSCI_B1_I2C_Init()
{
    //Disables the module to allow for initialization.

    //16 bits for the EUSCI_B1
    //FEDC_BA98_7654_3210

    EUSCI_B1->CTLW0 |= 0x0001;

    //Set to have 7bit addressing
    EUSCI_B1->CTLW0 &= ~0xC000;

    //Single master mode, basically saying the board is the "master"
    EUSCI_B1->CTLW0 &= ~0x2000;

    //Operate in master mode
    EUSCI_B1->CTLW0 |= 0x0800;

    //Use I2C
    EUSCI_B1->CTLW0 |= 0x0600;

    //Synchronous mode
    EUSCI_B1->CTLW0 |= 0x0100;

    //Use SMLCK which is 12MHz
    EUSCI_B1->CTLW0 |= 0x00C0;

    //Don't use the ACK condition
    EUSCI_B1->CTLW0 &= ~0x0020;

    //I2C master receiver mode
    EUSCI_B1->CTLW0 &= ~0x0010;

    //Clear bit as we aren't using slave receiver
    EUSCI_B1->CTLW0 &= ~0x0008;

    //Don't generate stop in master mode
    EUSCI_B1->CTLW0 &= ~0x0004;

    //Don't generate start in master mode

    EUSCI_B1->CTLW0 &= ~0x0002;

    //Clear all bits in CTLW1

    EUSCI_B1->CTLW1 &= ~0x01FF;

    //Desired freq for pwm driver is 1MHz

    // N = (Clock Frequency) / (SCL Frequency) = (12,000,000 / 1,000,000) = 12
    EUSCI_B1->BRW = 12;

    //Set the pins for:
    //  - SDA (P6.4)
    //  - SDL (P6.5)
    //Set primary module function (I2C) by setting SEL0 and clearing SEL1
    P6->SEL0 |= 0x30;  //only set pins 4-5
    P6->SEL1 &= ~0x30; //only clear pins 4-5

    //No interrupts
    EUSCI_B1->IE &= ~0x7FFF;

    //Take the module out of reset mode
    EUSCI_B1->CTLW0 &= ~0x0001;


}


void EUSCI_B1_I2C_Send_A_Byte(uint8_t slave_address, uint8_t data)
{
    while((EUSCI_B1->STATW & 0x0010) != 0);

    EUSCI_B1->I2CSA = slave_address;

    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0004) | 0x0012;

    while((EUSCI_B1->IFG & 0x0002) == 0);

    EUSCI_B1->TXBUF = data;

    while((EUSCI_B1->IFG & 0x0002) == 0);

    EUSCI_B1->CTLW0 |= 0x0004;

    EUSCI_B1->IFG &= ~0x0002;
}

void EUSCI_B1_I2C_Send_Multiple_Bytes(uint8_t slave_address, uint8_t *data_buffer, uint32_t packet_length)
{
    //Checks if it is busy. If not, proceed
    while((EUSCI_B1->STATW & 0x0010) != 0);

    //Sets the slave address
    EUSCI_B1->I2CSA = slave_address;

    //Set the module into master transmitter mode using UCTR bit(Bit 4).
    //Don't generate a stop(Bit 2).
    //Generate a start(Bit 1).
    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0004) | 0x0012;

    int i;
    for(i = 0; i < packet_length; i++)
    {
        //wait for transmitter to not be pending.
        while((EUSCI_B1->IFG & 0x0002) == 0);

        //stores a byte of data at a time in the Transmit Buffer.
        EUSCI_B1->TXBUF = data_buffer[i];
    }


    //wait for transmitter to not be pending.
    while((EUSCI_B1->IFG & 0x0002) == 0);

    //Generate a stop bit.
    EUSCI_B1->CTLW0 |= 0x0004;

    //Clear the transmit interrupt flag
    EUSCI_B1->IFG &= ~0x0002;
}


uint8_t EUSCI_B1_I2C_Receive_A_Byte(uint8_t slave_address)
{
    while((EUSCI_B1->STATW & 0x0010) != 0);

    EUSCI_B1->CTLW0 |= 0x0001;

    EUSCI_B1->TBCNT |= 0x0001;

    EUSCI_B1->CTLW0 &= ~0x0001;

    EUSCI_B1->I2CSA = slave_address;

    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0010) | 0x0006;

    while((EUSCI_B1->IFG & 0x0001) == 0);

    return ((uint8_t)(EUSCI_B1->RXBUF));
}

void EUSCI_B1_I2C_Receive_Multiple_Bytes(uint8_t slave_address, uint8_t *data_buffer, uint16_t packet_length)
{
    //Sets the slave address
    EUSCI_B1->I2CSA = slave_address;

    //Set the module into master receiver mode using UCTR bit(Bit 4).
    //Generate a start(Bit 1).
    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0010) | 0x0002;
    int i;
    for(i = 0; i < packet_length; i++)
    {
        //If last byte is being received, send stop condition
        if(i == (packet_length - 1))
        {
            EUSCI_B1->CTLW0 |= 0x0004;
        }

        //wait for receiver to not be pending
        while((EUSCI_B1->IFG & 0x0001) == 0);

        //stores a byte of data at a time in the Transmit Buffer.
        data_buffer[i] = EUSCI_B1->RXBUF;
    }

    //Wait until the Stop condition is transmitted.
    while((EUSCI_B1->CTLW0 & 0x0004) != 0);
}
