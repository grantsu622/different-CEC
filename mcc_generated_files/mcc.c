/**
  @Generated MPLAB(c) Code Configurator Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.c

  @Summary:
    This is the mcc.c file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC16F1782
        Driver Version    :  1.02
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

// Configuration bits: selected in the GUI

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection->INTOSC oscillator: I/O function on CLKIN pin
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable->WDT controlled by the SWDTEN bit in the WDTCON register
#pragma config PWRTE = ON    // Power-up Timer Enable->PWRT enabled
#pragma config MCLRE = ON    // MCLR Pin Function Select->MCLR/VPP pin function is MCLR
#pragma config CP = ON    // Flash Program Memory Code Protection->Program memory code protection is enabled
#pragma config CPD = OFF    // Data Memory Code Protection->Data memory code protection is disabled
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset enabled
#pragma config CLKOUTEN = OFF    // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config IESO = OFF    // Internal/External Switchover->Internal/External Switchover mode is disabled
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config VCAPEN = OFF    // Voltage Regulator Capacitor Enable bit->Vcap functionality is disabled on RA6.
#pragma config PLLEN = OFF    // PLL Enable->4x PLL disabled
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config BORV = LO    // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.
#pragma config LPBOR = ON    // Low Power Brown-Out Reset Enable Bit->Low power brown-out is enabled
#pragma config LVP = OFF    // Low-Voltage Programming Enable->Low-voltage programming enabled

#include "mcc.h"

void SYSTEM_Initialize(void)
{
    
    PIN_MANAGER_Initialize();
    OSCILLATOR_Initialize();
    TMR2_Initialize();
    ADC_Initialize();
    TMR0_Initialize();
}

void OSCILLATOR_Initialize(void)
{
    // SCS INTOSC; SPLLEN disabled; IRCF 16MHz_HF; 
    OSCCON = 0x7A;
    // LFIOFR disabled; HFIOFL not stable; OSTS intosc; PLLR disabled; HFIOFS not stable; HFIOFR disabled; MFIOFR disabled; T1OSCR disabled; 
    OSCSTAT = 0x00;
    // TUN 0; 
    OSCTUNE = 0x00;
    // Set the secondary oscillator
    
}

/**
 End of File
*/
