/**
  TMR0 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr0.c

  @Summary
    This is the generated driver implementation file for the TMR0 driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for TMR0.
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

/**
  Section: Included Files
*/

#include <xc.h>
#include "tmr0.h"

/**
  Section: Global Variables Definitions
*/
static volatile uint8_t _u8ms1Counter;
static volatile uint8_t _u8ms10Counter;
uint8_t gu8TMR0State;

/**
  Section: TMR0 APIs
  16MHz/4/32=0.125MHz
  1/0.125(us)*132=1.056ms
*/

void TMR0_Initialize(void)
{
    // Set TMR0 to the options selected in the User Interface
    gu8TMR0State = 0x00;
    _u8ms1Counter = 0;
    _u8ms10Counter = 0;

    // PSA assigned; PS 1:32; TMRSE Increment_hi_lo; mask the nWPUEN and INTEDG bits
    OPTION_REG = (OPTION_REG & 0xC0) | 0xD4 & 0x3F; 

    // TMR0 131; 
    TMR0 = 0x83;

    // Clear Interrupt flag before enabling the interrupt
    INTCONbits.TMR0IF = 0;

    // Enabling TMR0 interrupt
    INTCONbits.TMR0IE = 1;
}

void TMR0_ISR(void)
{

    // clear the TMR0 interrupt flag
    INTCONbits.TMR0IF = 0;

    // TMR0 131; 
    TMR0 = 0x83;

    // ticker function call;
    // ticker is 1 -> Callback function gets called everytime this ISR executes
    TMR0_CallBack();

    // add your TMR0 interrupt custom code
}

void TMR0_CallBack(void)
{
    // Add your custom callback code here
    // this code executes every 1 TMR0 periods
    
    _u8ms1Counter++;
	// Period = 2 ms
	if ( (_u8ms1Counter % 2) == 0 )
		gu8TMR0State |= TMR0_FLAG_2MS;

    if (_u8ms1Counter == 10)
    {
        _u8ms1Counter = 0;
        _u8ms10Counter++;
        gu8TMR0State |= TMR0_FLAG_10MS;

        // Period = 50 ms
        if ( (_u8ms10Counter % 5) == 0 )
            gu8TMR0State |= TMR0_FLAG_50MS;

        // Period = 100 ms
        if ( (_u8ms10Counter % 10) == 0 )
        {
            gu8TMR0State |= TMR0_FLAG_100MS;
    	}
        
        // Period = 500 ms
        if ( (_u8ms10Counter % 50) == 0 )
        {
            gu8TMR0State |= TMR0_FLAG_500MS;
    	}

        // Period = 1000 ms
        if ( _u8ms10Counter >= 100 )
        {
            gu8TMR0State |= TMR0_FLAG_1S;
            _u8ms10Counter = 0; // reset ms counter
        }
    }
}

void TMR0_ClearCounter(void)
{
    _u8ms1Counter = 0;
    _u8ms10Counter = 0;
    gu8TMR0State = 0x00;
}
/**
  End of File
*/
