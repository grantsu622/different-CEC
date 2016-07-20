/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC16F1782
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_W_CTRL aliases
#define IO_W_CTRL_TRIS               TRISA3
#define IO_W_CTRL_LAT                LATA3
#define IO_W_CTRL_PORT               RA3
#define IO_W_CTRL_WPU                WPUA3
#define IO_W_CTRL_ANS                ANSA3
#define IO_W_CTRL_SetHigh()    do { LATA3 = 1; } while(0)
#define IO_W_CTRL_SetLow()   do { LATA3 = 0; } while(0)
#define IO_W_CTRL_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define IO_W_CTRL_GetValue()         RA3
#define IO_W_CTRL_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define IO_W_CTRL_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define IO_W_CTRL_SetPullup()    do { WPUA3 = 1; } while(0)
#define IO_W_CTRL_ResetPullup()   do { WPUA3 = 0; } while(0)
#define IO_W_CTRL_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define IO_W_CTRL_SetDigitalMode()   do { ANSA3 = 0; } while(0)


// get/set IO_B_CTRL aliases
#define IO_B_CTRL_TRIS               TRISA4
#define IO_B_CTRL_LAT                LATA4
#define IO_B_CTRL_PORT               RA4
#define IO_B_CTRL_WPU                WPUA4
#define IO_B_CTRL_ANS                ANSA4
#define IO_B_CTRL_SetHigh()    do { LATA4 = 1; } while(0)
#define IO_B_CTRL_SetLow()   do { LATA4 = 0; } while(0)
#define IO_B_CTRL_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define IO_B_CTRL_GetValue()         RA4
#define IO_B_CTRL_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define IO_B_CTRL_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define IO_B_CTRL_SetPullup()    do { WPUA4 = 1; } while(0)
#define IO_B_CTRL_ResetPullup()   do { WPUA4 = 0; } while(0)
#define IO_B_CTRL_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define IO_B_CTRL_SetDigitalMode()   do { ANSA4 = 0; } while(0)


// get/set IO_Y_CTRL aliases
#define IO_Y_CTRL_TRIS               TRISA5
#define IO_Y_CTRL_LAT                LATA5
#define IO_Y_CTRL_PORT               RA5
#define IO_Y_CTRL_WPU                WPUA5
#define IO_Y_CTRL_ANS                ANSA5
#define IO_Y_CTRL_SetHigh()    do { LATA5 = 1; } while(0)
#define IO_Y_CTRL_SetLow()   do { LATA5 = 0; } while(0)
#define IO_Y_CTRL_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define IO_Y_CTRL_GetValue()         RA5
#define IO_Y_CTRL_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define IO_Y_CTRL_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

#define IO_Y_CTRL_SetPullup()    do { WPUA5 = 1; } while(0)
#define IO_Y_CTRL_ResetPullup()   do { WPUA5 = 0; } while(0)
#define IO_Y_CTRL_SetAnalogMode()   do { ANSA5 = 1; } while(0)
#define IO_Y_CTRL_SetDigitalMode()   do { ANSA5 = 0; } while(0)


// get/set W_Signal aliases
#define W_Signal_TRIS               TRISB1
#define W_Signal_LAT                LATB1
#define W_Signal_PORT               RB1
#define W_Signal_WPU                WPUB1
#define W_Signal_ANS                ANSB1
#define W_Signal_SetHigh()    do { LATB1 = 1; } while(0)
#define W_Signal_SetLow()   do { LATB1 = 0; } while(0)
#define W_Signal_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define W_Signal_GetValue()         RB1
#define W_Signal_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define W_Signal_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define W_Signal_SetPullup()    do { WPUB1 = 1; } while(0)
#define W_Signal_ResetPullup()   do { WPUB1 = 0; } while(0)
#define W_Signal_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define W_Signal_SetDigitalMode()   do { ANSB1 = 0; } while(0)


// get/set B_Signal aliases
#define B_Signal_TRIS               TRISB2
#define B_Signal_LAT                LATB2
#define B_Signal_PORT               RB2
#define B_Signal_WPU                WPUB2
#define B_Signal_ANS                ANSB2
#define B_Signal_SetHigh()    do { LATB2 = 1; } while(0)
#define B_Signal_SetLow()   do { LATB2 = 0; } while(0)
#define B_Signal_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define B_Signal_GetValue()         RB2
#define B_Signal_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define B_Signal_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define B_Signal_SetPullup()    do { WPUB2 = 1; } while(0)
#define B_Signal_ResetPullup()   do { WPUB2 = 0; } while(0)
#define B_Signal_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define B_Signal_SetDigitalMode()   do { ANSB2 = 0; } while(0)


// get/set Y_Signal aliases
#define Y_Signal_TRIS               TRISB3
#define Y_Signal_LAT                LATB3
#define Y_Signal_PORT               RB3
#define Y_Signal_WPU                WPUB3
#define Y_Signal_ANS                ANSB3
#define Y_Signal_SetHigh()    do { LATB3 = 1; } while(0)
#define Y_Signal_SetLow()   do { LATB3 = 0; } while(0)
#define Y_Signal_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define Y_Signal_GetValue()         RB3
#define Y_Signal_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define Y_Signal_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define Y_Signal_SetPullup()    do { WPUB3 = 1; } while(0)
#define Y_Signal_ResetPullup()   do { WPUB3 = 0; } while(0)
#define Y_Signal_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define Y_Signal_SetDigitalMode()   do { ANSB3 = 0; } while(0)


// get/set IO_CONTROL aliases
#define IO_CONTROL_TRIS               TRISB4
#define IO_CONTROL_LAT                LATB4
#define IO_CONTROL_PORT               RB4
#define IO_CONTROL_WPU                WPUB4
#define IO_CONTROL_ANS                ANSB4
#define IO_CONTROL_SetHigh()    do { LATB4 = 1; } while(0)
#define IO_CONTROL_SetLow()   do { LATB4 = 0; } while(0)
#define IO_CONTROL_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define IO_CONTROL_GetValue()         RB4
#define IO_CONTROL_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define IO_CONTROL_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define IO_CONTROL_SetPullup()    do { WPUB4 = 1; } while(0)
#define IO_CONTROL_ResetPullup()   do { WPUB4 = 0; } while(0)
#define IO_CONTROL_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define IO_CONTROL_SetDigitalMode()   do { ANSB4 = 0; } while(0)


// get/set IO_FEEDBACK aliases
#define IO_FEEDBACK_TRIS               TRISB5
#define IO_FEEDBACK_LAT                LATB5
#define IO_FEEDBACK_PORT               RB5
#define IO_FEEDBACK_WPU                WPUB5
#define IO_FEEDBACK_ANS                ANSB5
#define IO_FEEDBACK_SetHigh()    do { LATB5 = 1; } while(0)
#define IO_FEEDBACK_SetLow()   do { LATB5 = 0; } while(0)
#define IO_FEEDBACK_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define IO_FEEDBACK_GetValue()         RB5
#define IO_FEEDBACK_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define IO_FEEDBACK_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define IO_FEEDBACK_SetPullup()    do { WPUB5 = 1; } while(0)
#define IO_FEEDBACK_ResetPullup()   do { WPUB5 = 0; } while(0)
#define IO_FEEDBACK_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define IO_FEEDBACK_SetDigitalMode()   do { ANSB5 = 0; } while(0)


// get/set IO_CW_GREEN aliases
#define IO_CW_GREEN_TRIS               TRISC1
#define IO_CW_GREEN_LAT                LATC1
#define IO_CW_GREEN_PORT               RC1
#define IO_CW_GREEN_WPU                WPUC1
#define IO_CW_GREEN_SetHigh()    do { LATC1 = 1; } while(0)
#define IO_CW_GREEN_SetLow()   do { LATC1 = 0; } while(0)
#define IO_CW_GREEN_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define IO_CW_GREEN_GetValue()         RC1
#define IO_CW_GREEN_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define IO_CW_GREEN_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

#define IO_CW_GREEN_SetPullup()    do { WPUC1 = 1; } while(0)
#define IO_CW_GREEN_ResetPullup()   do { WPUC1 = 0; } while(0)


// get/set IO_CCW_RED aliases
#define IO_CCW_RED_TRIS               TRISC2
#define IO_CCW_RED_LAT                LATC2
#define IO_CCW_RED_PORT               RC2
#define IO_CCW_RED_WPU                WPUC2
#define IO_CCW_RED_SetHigh()    do { LATC2 = 1; } while(0)
#define IO_CCW_RED_SetLow()   do { LATC2 = 0; } while(0)
#define IO_CCW_RED_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define IO_CCW_RED_GetValue()         RC2
#define IO_CCW_RED_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define IO_CCW_RED_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define IO_CCW_RED_SetPullup()    do { WPUC2 = 1; } while(0)
#define IO_CCW_RED_ResetPullup()   do { WPUC2 = 0; } while(0)



/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
*/