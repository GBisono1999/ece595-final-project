/**
 * @file main.c
 * @brief Main source code for the GPIO program.
 *
 * This file contains the main entry point and function definitions for the GPIO program.
 * It interfaces with the following:
 *  - User buttons and LEDs of the TI MSP432 LaunchPad
 *  - PMOD SWT (4 Slide Switches)
 *  - PMOD 8LD (8 LEDs)
 *
 * To verify the pinout of the user buttons and LEDs, refer to the MSP432P401R SimpleLink Microcontroller LaunchPad Development Kit User's Guide
 * Link: https://docs.rs-online.com/3934/A700000006811369.pdf
 *
 * For more information regarding the PMODs used in this lab, visit the following links:
 *  - PMOD SWT: https://digilent.com/reference/pmod/pmodswt/reference-manual
 *  - PMOD 8LD: https://digilent.com/reference/pmod/pmod8ld/reference-manual
 *
 * @note The user buttons, located at P1.1 and P1.4, are configured with negative logic
 * as the default setting. When the buttons are pressed, they connect to GND. Refer to the
 * schematic found in the MSP432P401R LaunchPad User's Guide.
 *
 * @author Aaron Nanas
 */

#include <stdint.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
#include "adafruit_pwm_driver/inc/pwm_driver.h"


static uint8_t servonum = 0;

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)

int main(void)
{
    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    PCA9685_Init();

    PCA9685_setPWMFreq(50);

    Clock_Delay1ms(10);

    while(1)
    {
        uint16_t pulselen;
        for (pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
            PCA9685_setPWM(servonum, 0, pulselen);
        }

        Clock_Delay1ms(500);

        for (pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
            PCA9685_setPWM(servonum, 0, pulselen);
        }
        Clock_Delay1ms(500);

        servonum++;
        if (servonum > 3) servonum = 0; //Testing the first four servo channels
    }
}
