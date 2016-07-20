/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC16F1782
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20
*/

/*
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

#include "mcc.h"
#include "tmr0.h"
#include "adc.h"

#define MODE_CHECK_VOLT     0x00
#define MODE_IN_VOLT        0x01
#define MODE_CHECK_CONTROL  0x02
#define MODE_CONTROL_HIGH   0x03    // Check ECU Control
#define MODE_CONTROL_LOW    0x04    // Check ECU Control
#define MODE_BACKWARD       0x05    // Check CCW
#define MODE_FORWARD        0x06    // Check CW
#define MODE_FINISH         0x07

#define DEF_OVP             3190    // 15.5V
#define DEF_UVP             2250    // 8.5V

volatile uint8_t gu8Mode; //globel uint8 Mode

void InitVariable(void);
void InitMotorPosition(void);
void RelayTimeout(void);

bool bCheckInTheMiddle(void);
bool bCheckBConnectW(void);

/*
                         Main application
 */
void main(void)
{
    uint16_t u16VoltVar;
    uint8_t u8CtrlHighCnt;
    uint8_t u8CtrlLowCnt;
    uint8_t u8SignalCnt;
    uint8_t u8VoltCnt;
    bool    bPowerIssue;
    bool    bCheckPower;
    
    // initialize the device
    SYSTEM_Initialize();
    InitVariable();
    
    u16VoltVar = 0;
    u8CtrlHighCnt = 0;
    u8CtrlLowCnt = 0;
    u8SignalCnt = 0;
    u8VoltCnt = 0;
    bPowerIssue = false;
    bCheckPower = false;

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    __delay_ms(100);
    
    while (1)
    {
        if( gu8Mode != MODE_CHECK_VOLT && bPowerIssue == false )
        {
            if( bCheckPower == false )
            {
                if( gu8TMR0State & TMR0_FLAG_1S )
                {
                    gu8TMR0State &= ~TMR0_FLAG_1S;

                    u16VoltVar = 0;
                    u16VoltVar = ADC_GetVolt();
                    // 8.5V = 2250, 12V = 2850, 14V = 3200 1V ~= 170
                    if( u16VoltVar > DEF_OVP || u16VoltVar < DEF_UVP )
                    {
                        // Issue
                        u8VoltCnt = 0;
                        bCheckPower = true;
                    }
                    else
                    {
                        bPowerIssue = false;
                    }
                }
            }
            else
            {
                if( gu8TMR0State & TMR0_FLAG_10MS )
                {
                    gu8TMR0State &= ~TMR0_FLAG_10MS;
                    
                    u16VoltVar = 0;
                    u16VoltVar = ADC_GetVolt();
                    // To-Do:
                    // 8.5V = 2250, 12V = 2850, 16V = 3530 1V ~= 170
                    if( u16VoltVar > DEF_OVP || u16VoltVar < DEF_UVP )
                    {
                        u8VoltCnt++;
                        if( u8VoltCnt > 5 )
                        {
                            bPowerIssue = true;
                            bCheckPower = false;
                        }
                    }
                    else
                    {
                        u8VoltCnt = 0;
                        bCheckPower = false;
                    }
                }
            }
        }
        
        // Add your application code
        switch( gu8Mode )
        {
            case MODE_CHECK_VOLT:
                if( gu8TMR0State & TMR0_FLAG_10MS )
                {
                    gu8TMR0State &= ~TMR0_FLAG_10MS;
                    
                    u16VoltVar = 0;
                    u16VoltVar = ADC_GetVolt();
                    // To-Do:
                    // 8.5V = 2250, 12V = 2850, 16V = 3530 1V ~= 170
                    if( u16VoltVar > DEF_OVP || u16VoltVar < DEF_UVP )
                    {
                        u8VoltCnt++;
                        if( u8VoltCnt > 5 )
                        {
                            bPowerIssue = true;
                        }
                    }
                    else
                    {
                        u8VoltCnt = 0;
                        InitMotorPosition();
                    }
                }
                break;
                
            case MODE_IN_VOLT:
                break;
                
            case MODE_CHECK_CONTROL:
                if( gu8TMR0State & TMR0_FLAG_10MS )
                {
                    gu8TMR0State &= ~TMR0_FLAG_10MS;
                    u16VoltVar = 0;
                    u16VoltVar = ADC_GetVolt();
                    // To-Do:
                    // 8.5V = 2250, 12V = 2850, 16V = 3530 1V ~= 170
                    if( u16VoltVar > DEF_OVP || u16VoltVar < DEF_UVP )
                    {
                        u8VoltCnt++;
                        if( u8VoltCnt > 5 )
                        {
                            gu8Mode = MODE_CHECK_VOLT;
                        }
                    }
                    else
                    {
                        if( IO_CONTROL_GetValue() == HIGH )
                        {
                            u8CtrlHighCnt++;
                            u8CtrlLowCnt = 0;
                            if( u8CtrlHighCnt > 5 )
                            {
                                u8CtrlHighCnt = 0;
                                u8CtrlLowCnt = 0;
                                u8SignalCnt = 0;
                                gu8Mode = MODE_CONTROL_HIGH;
                            }
                        }
                        else
                        {
                            u8CtrlHighCnt = 0;
                            u8CtrlLowCnt++;
                            if( u8CtrlLowCnt > 5 )
                            {
                                u8CtrlHighCnt = 0;
                                u8CtrlLowCnt = 0;
                                u8SignalCnt = 0;
                                gu8Mode = MODE_CONTROL_LOW;
                            }
                        }
                    }
                }
                break;
                
            case MODE_CONTROL_HIGH:
                if( gu8TMR0State & TMR0_FLAG_10MS )
                {
                    gu8TMR0State &= ~TMR0_FLAG_10MS;
                    
                    IO_FEEDBACK_SetDigitalOutput();
                    if( IO_FEEDBACK_GetValue() == HIGH )
                    {
                        // C1F1
                        if( u8SignalCnt < 5 )
                        {
                            u8SignalCnt++;
                        }
                        else
                        {
                            gu8Mode = MODE_FORWARD;    // CW
                            TMR2_StartTimer();
                            TMR2_WriteTimer(0);
                            IO_CW_GREEN_SetHigh();
                        }
                    }
                    else
                    {
                        // C1F0
                        u8SignalCnt = 0;
                        u8CtrlHighCnt = 0;
                        u8CtrlLowCnt = 0;
                        gu8Mode = MODE_CHECK_CONTROL;
                    }
                }
                break;
                
            case MODE_CONTROL_LOW:
                if( gu8TMR0State & TMR0_FLAG_10MS )
                {
                    gu8TMR0State &= ~TMR0_FLAG_10MS;
                    
                    IO_FEEDBACK_SetDigitalOutput();
                    if( IO_FEEDBACK_GetValue() == LOW )
                    {
                        // C0F0
                        if( u8SignalCnt < 5 )
                        {
                            u8SignalCnt++;
                        }
                        else
                        {
                            gu8Mode = MODE_BACKWARD;    // CCW
                            TMR2_StartTimer();
                            TMR2_WriteTimer(0);
                            IO_CCW_RED_SetHigh();
                        }
                    }
                    else
                    {
                        // C0F1
                        u8CtrlHighCnt = 0;
                        u8CtrlLowCnt = 0;
                        u8SignalCnt = 0;
                        gu8Mode = MODE_CHECK_CONTROL;
                    }
                }
                break;
                
            case MODE_FORWARD:
                if( LOW == IO_CW_GREEN_GetValue() ||
                    true == bCheckInTheMiddle() )
                {
                    TMR2_StopTimer();
                    IO_CW_GREEN_SetLow();
                    IO_B_CTRL_SetHigh();        // Set B signal to low
                    IO_Y_CTRL_SetLow();        // Set Y signal to high
                    IO_W_CTRL_SetHigh();        // Set W signal to low
                    
                    IO_FEEDBACK_SetDigitalOutput();
                    IO_FEEDBACK_SetLow();
                    u8CtrlHighCnt = 0;
                    u8CtrlLowCnt = 0;
                    if( bPowerIssue == true )
                    {
                        bPowerIssue = false;
                        gu8Mode = MODE_CHECK_VOLT;
                    }
                    else
                    {
                        gu8Mode = MODE_CHECK_CONTROL;
                    }
                }
                break;
                
            case MODE_BACKWARD:
                if( LOW == IO_CCW_RED_GetValue() ||
                    true == bCheckBConnectW() )
                {
                    TMR2_StopTimer();
                    IO_CCW_RED_SetLow();
                    IO_B_CTRL_SetHigh();        // Set B signal to low
                    IO_Y_CTRL_SetLow();        // Set Y signal to high
                    IO_W_CTRL_SetHigh();        // Set W signal to low
                    
                    IO_FEEDBACK_SetDigitalOutput();
                    IO_FEEDBACK_SetHigh();
                    u8CtrlHighCnt = 0;
                    u8CtrlLowCnt = 0;
                    if( bPowerIssue == true )
                    {
                        bPowerIssue = false;
                        gu8Mode = MODE_CHECK_VOLT;
                    }
                    else
                    {
                        gu8Mode = MODE_CHECK_CONTROL;
                    }
                }
                break;
                
            case MODE_FINISH:
                IO_CW_GREEN_SetLow();
                IO_CCW_RED_SetLow();
                IO_B_CTRL_SetHigh();        // Set B signal to low
                IO_Y_CTRL_SetLow();        // Set Y signal to high
                IO_W_CTRL_SetHigh();        // Set W signal to low
                TMR2_StopTimer();
                
                IO_FEEDBACK_SetDigitalOutput();
                if( IO_CONTROL_GetValue() == LOW )
                {
                    IO_FEEDBACK_SetHigh();
                }
                else
                {
                    IO_FEEDBACK_SetLow();
                }
                u8CtrlHighCnt = 0;
                u8CtrlLowCnt = 0;
                if( bPowerIssue == true )
                {
                    bPowerIssue = false;
                    gu8Mode = MODE_CHECK_VOLT;
                }
                else
                {
                    gu8Mode = MODE_CHECK_CONTROL;
                }
                break;
                
            default:
                break;
        }
        
        CLRWDT();
    }
    
}

void InitVariable(void)
{
    WDTCONbits.SWDTEN = 0;
    gu8Mode = MODE_CHECK_VOLT;
    
    // Stop Relay
    IO_CW_GREEN_SetLow();
    IO_CCW_RED_SetLow();
    
    IO_B_CTRL_SetHigh();
    IO_Y_CTRL_SetLow();
    IO_W_CTRL_SetHigh();
    
    // Set Timeout Handler
    TMR2_SetInterruptHandler(RelayTimeout);
}

void InitMotorPosition(void)
{
    gu8Mode = MODE_CHECK_CONTROL;
    
    IO_FEEDBACK_SetDigitalOutput();
    if( IO_CONTROL_GetValue() == LOW )
    {
        if( true == bCheckBConnectW() )
        {
            IO_FEEDBACK_SetHigh();
        }
        else
        {
            IO_FEEDBACK_SetLow();
        }
    }
    else
    {
        if( true == bCheckInTheMiddle() )
        {
            IO_FEEDBACK_SetLow();
        }
        else
        {
            IO_FEEDBACK_SetHigh();
        }
    }
    IO_FEEDBACK_SetDigitalInput();
}

bool bCheckInTheMiddle(void)
{
    uint16_t u16YVar,u16BVar,u16WVar;
    
    IO_B_CTRL_SetHigh();
    IO_Y_CTRL_SetLow();
    IO_W_CTRL_SetHigh();
    ADC_GetVolt();
    
    u16BVar = 0;
    u16BVar = ADC_GetBSignal();

    u16YVar = 0;
    u16YVar = ADC_GetYSignal();

    u16WVar = 0;
    u16WVar = ADC_GetWSignal();
    
    if( u16BVar >= 3000 &&
        u16WVar < 1000 &&
        u16YVar < 1000 )
    {
        // In Middle
        return true;
    }
    return false;
}

bool bCheckBConnectW(void)
{
    uint16_t u16YVar,u16BVar,u16WVar;
    
    IO_B_CTRL_SetHigh();
    IO_Y_CTRL_SetLow();
    IO_W_CTRL_SetHigh();
    ADC_GetVolt();

    u16BVar = 0;
    u16BVar = ADC_GetBSignal();

    u16YVar = 0;
    u16YVar = ADC_GetYSignal();

    u16WVar = 0;
    u16WVar = ADC_GetWSignal();
    
    if( u16YVar >= 3000 && 
        u16BVar < 1000 && 
        u16WVar < 1000 )          // W connect B
    {
        return true;
    }
    return false;
}

void RelayTimeout(void) //2s
{
    IO_CW_GREEN_SetLow();
    IO_CCW_RED_SetLow();
    TMR2_StopTimer();
}


/**
 End of File
*/