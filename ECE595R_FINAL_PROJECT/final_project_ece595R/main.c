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
 * schematic found in the MSP432P401R LaunchPad User's Guide .
 *
 * @author Aaron Nanas
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
#include "adafruit_pwm_driver/inc/pwm_driver.h"
#include "inc/BLE_UART.h"


static uint8_t servonum = 0;

static uint16_t pulselen = 0;

#define SERVOMIN  200 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  400 // This is the 'maximum' pulse length count (out of 4096)

static uint16_t base_servo_value_r = 0;
static uint16_t base_servo_value_l = 0;

static uint16_t arm_servo_value = 0;
static uint16_t wrist_servo_value = 0;

static uint16_t claw_rotation_servo_value = 0;
static uint16_t claw_mechanism_servo_value = 0;

static bool activate_servo_commands = true;
static bool activate_advanced_servo_commands = false;

//Implement if you have time
static bool activate_controller = false;


void Process_BLE_UART_Data(char BLE_UART_Buffer[])
{

    //MAKE SURE THE SERVOS TAKE A BIT OF TIME BEFORE HITTING THE DESIRED VALUE
    //THIS IS TO HELP PREVENT STRIPPING OF THE SERVO HORNS
    if(activate_servo_commands)
    {
        //base servos
        if(Check_BLE_UART_Data(BLE_UART_Buffer, "BASE MID"))
        {
            base_servo_value_r = 300;
            base_servo_value_l = 300;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "BASE FWD"))
        {
            base_servo_value_r = 400;
            base_servo_value_l = 200;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "BASE BWD"))
        {
            base_servo_value_r = 200;
            base_servo_value_l = 400;
        }

        //arm servo
        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ARM MID"))
        {
            arm_servo_value = 300;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ARM FWD"))
        {
            arm_servo_value = 450;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ARM BWD"))
        {
            arm_servo_value = 150;
        }

        //wrist servo
        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "WRS MID"))
        {
            wrist_servo_value = 300;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "WRS FWD"))
        {
            wrist_servo_value = 150;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "WRS BWD"))
        {
            wrist_servo_value = 450;
        }


        //claw_rotation_servo_value
        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ROT MID"))
        {
            claw_rotation_servo_value = 300;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ROT FWD"))
        {
            claw_rotation_servo_value = 150;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ROT BWD"))
        {
            claw_rotation_servo_value = 450;
        }


        //claw mechanism
        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "OPEN CLAW"))
        {
            claw_mechanism_servo_value = 400;
        }

        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "CLOSE CLAW"))
        {
            claw_mechanism_servo_value = 100;
        }
    }

    if(activate_advanced_servo_commands)
    {
        if(Check_BLE_UART_Data(BLE_UART_Buffer, "PILLAR"))
        {
            base_servo_value_r = 300;
            base_servo_value_l = 300;
            Clock_Delay1ms(1000);
            arm_servo_value = 300;
            Clock_Delay1ms(1000);
            wrist_servo_value = 300;
            Clock_Delay1ms(1000);
        }
        else if(Check_BLE_UART_Data(BLE_UART_Buffer, "FWD L"))
        {
            base_servo_value_r = 300;
            base_servo_value_l = 300;
            Clock_Delay1ms(1000);
            arm_servo_value = 450;
            Clock_Delay1ms(1000);
            wrist_servo_value = 150;
            Clock_Delay1ms(1000);
            claw_mechanism_servo_value = 400;
        }
        //predefined positions to "speed" up actions

        //possible commands
        //1) change positon to desired position and close claw, pick up command
        //2) "stand" straight up
    }


    if(Check_BLE_UART_Data(BLE_UART_Buffer, "RGB LED GREEN"))
    {
        LED2_Output(RGB_LED_GREEN);
    }

    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "RGB LED OFF"))
    {
        LED2_Output(RGB_LED_OFF);
    }

    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ENABLE SERVO COMMANDS"))
    {
        activate_servo_commands = true;
    }

    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "DISABLE SERVO COMMANDS"))
    {
        activate_servo_commands = false;
    }

    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ENABLE ADV SERVO COMMANDS"))
    {
        activate_advanced_servo_commands = true;
    }

    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "DISABLE ADV SERVO COMMANDS"))
    {
        activate_advanced_servo_commands = false;
    }
}

int main(void)
{
    // Initialize the 48 MHz Clock
    Clock_Init48MHz();
    EUSCI_A0_UART_Init_Printf();

    LED2_Init();

    BLE_UART_Init();

    char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};

    BLE_UART_Reset();
    BLE_UART_OutString("BLE UART Active\r\n");


    printf("BEGIN\n");

    //EUSCI_B1_I2C_Init();

    //EUSCI_A0_UART_Init();
    PCA9685_Init();

    PCA9685_setPWMFreq(50);
    Clock_Delay1ms(10);

    printf("END\n");

    Clock_Delay1ms(1000);

    while(1)
    {
        //PCA9685_setPWM(8, 0, 300 + 0);
        //PCA9685_setPWM(9, 0, 300 - 0); //Right servo


        PCA9685_setPWM(8, 0, base_servo_value_l);
        PCA9685_setPWM(9, 0, base_servo_value_r); //Right servo
//claw_mechanism_servo_value
        PCA9685_setPWM(12, 0, arm_servo_value);
        //wrist_servo_value
        PCA9685_setPWM(13, 0, wrist_servo_value);

        //claw_rotation_servo_value

        PCA9685_setPWM(14, 0, claw_rotation_servo_value);
        PCA9685_setPWM(15, 0, claw_mechanism_servo_value);

        //PCA9685_setPWM(10, 0, 300 - 0);
        int string_size = BLE_UART_InString(BLE_UART_Buffer, BLE_UART_BUFFER_SIZE);

        printf("BLE UART Data: ");

        int i = 0;

        for(i = 0; i < string_size; i = i + 1)
        {
            printf("%c", BLE_UART_Buffer[i]);
        }

        printf("\n");

        Process_BLE_UART_Data(BLE_UART_Buffer);

    }
}
