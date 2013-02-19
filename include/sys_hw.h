/*****************************************************************************
*                     Copyright (c) 2011, Marcello Bonfe'                     
*                   ENDIF - ENgineering Department In Ferrara,
*                           University of Ferrara.
*                            All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions 
*  are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of the University of Ferrara nor the
*      names of its contributors may be used to endorse or promote products
*      derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
*  ARE DISCLAIMED. IN NO EVENT SHALL MARCELLO BONFE' BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
*  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
* 
******************************************************************************
 *                                                                *
 *    Author: Marcello Bonfe'                                     *
 *                                                                *
 *    Filename:       sys_hw.h                                    *
 *    Date:           20/04/2011                                  *
 *    File Version:   0.2                                         *
 *    Compiler:       MPLAB C30 v3.23                             *
 *                                                                *
 ******************************************************************
 *    Code Description
 *  
 *  This file contains system and hardware specific definitions.
 * 
 ********************************************************************/
#ifndef SYS_HW_H
#define SYS_HW_H
 
#include <p33Fxxxx.h>



/****************************************************************
 * DEFINITIONS TO ADAPT BOARD SPECIFIC OPTIONS OR SIMULATIONS
 ***************************************************************/
//LEAVE UNCOMMENTED ONLY ONE OF THE FOLLOWING
//for use mplab sim
//#define SIMULATE
#define PROTO_BOARD

/****************************************************************
 * IF COMPILED FOR DEVELOP MODE
 * ADDS DATALOGS AND TEST PROBE OUTPUTS FOR TASK MONITORING
 ***************************************************************/
//#define DEVELOP_MODE
//UNCOMMENT THE FOLLOWING IF LONG (data-type) DATALOG IS DESIRED
#define LOG_LONG
//LEAVE UNCOMMENTED ONLY ONE OF THE FOLLOWING
//#define LOG_POSLOOP
#define LOG_TRACKLOOP
//#define LOG_ADCINT

/*******************************************************************
 * System Clock Timing -
 * For oscillator configuration FRC (7.37MHz x PLL config),
 * Device Throughput in MIPS = Fcy = 7370000*40 / 2 / 2 / 2 = ~36.85 MIPS
 * Instruction Cycle time = Tcy = 1/(Fcy) = ~27 nanoseconds
 ******************************************************************/
#define FRC_FREQ    7370000     //On-board Crystal fcy
#define PLL_M       40         //On-chip PLL setting
#define PLL_N2      2
#define PLL_N1      2
//Instruction Cycle Fcy
#define FCY         FRC_FREQ*PLL_M / PLL_N2 / PLL_N1 / 2

/*******************************************************************
 * H-Bridge Control pins (PWM/DIR/BRAKE) 
 * and feedback (Thermal Flag, Current Sense)
 * for LMD18200
 ******************************************************************/
// define locked antiphase mode
//#define BRIDGE_LAP
// PWMx pins are "Module controlled", no need to set TRISx manually!
#define PWM1_TRIS TRISBbits.TRISB15
#define PWM2_TRIS TRISBbits.TRISB13
#define PWM3_TRIS TRISBbits.TRISB9
#endif
 
// Other pins are instead standard dig.outputs..
#define DIR1_TRIS TRISBbits.TRISB14
#define DIR2_TRIS TRISBbits.TRISB12
#define DIR3_TRIS TRISBbits.TRISB8
 
// NO BRAKES CAN BE USED
//#define BRAKE1_TRIS TRISAbits.TRISA3
//#define BRAKE2_TRIS TRISBbits.TRISB14
  
#define CURRSENSE1_TRIS TRISAbits.TRISA0
#define CURRSENSE2_TRIS TRISAbits.TRISA1
#define CURRSENSE3_TRIS TRISBbits.TRISB0
 
// Current sense analog pin PCFG (Port ConFiGuration)
// 1 - Digital, 0 - Analog
#define CURRSENSE1_PCFG AD1PCFGLbits.PCFG0 
#define CURRSENSE2_PCFG AD1PCFGLbits.PCFG1
#define CURRSENSE3_PCFG AD1PCFGLbits.PCFG2




#ifdef BRIDGE_LAP
// PWMx pins are controlled by duty-cycle generators, no need to use LATx
#define PWM1 LATBbits.LATB15 
#define PWM2 LATBbits.LATB13
#define PWM3 LATBbits.LATB9
#endif

// Other pins are instead standard digital I/Os
#define DIR1 LATBbits.LATB14
#define DIR2 LATBbits.LATB12 
#define DIR3 LATBbits.LATB8



//#define BRAKE1 LATAbits.LATA3 
//#define BRAKE2 LATBbits.LATB14 

/******************************************************************
* Encoders: QEI inputs and T1/T4 CLK
******************************************************************/

// QEI mode -> "Module controlled", no need to set TRISx manually

// T4CK is a on a remappable pin
// BUT FOR INPUT MAPPING (ONLY) PPS DOES NOT HAVE
// PRIORITY OVER TRIS!!!!!!!! NEED TO SET UP TRIS ALSO
#define T1CK_TRIS TRISAbits.TRISA4
#define T4CK_TRIS TRISBbits.TRISB3

/*******************************************************************
 * Communication channels (UART1-2, CAN, SPI)
 ******************************************************************/

// UART1: Tx -> RB5, Rx <- RB4
// UART2: 
// CAN:   
// I2C:   
// are all "Module controlled", no need to use TRISx/LATx

/*******************************************************************
 * ANALOG INPUTS
 ******************************************************************/
 
#define AN0_TRIS TRISAbits.TRISA0
#define AN1_TRIS TRISAbits.TRISA1 
#define AN2_TRIS TRISBbits.TRISB0
#define AN3_TRIS TRISBbits.TRISB1
#define AN4_TRIS TRISBbits.TRISB2
#define AN5_TRIS TRISBbits.TRISB3

/*******************************************************************
 * TEST OUTPUT
 ******************************************************************/

#define TEST_PIN_TRIS TRISAbits.TRISA2
#define TEST_PIN LATAbits.LATA2

/*************************************************************
 * add bridge for select lap or raw power
 *************************************************************/

