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
*****************************************************************************
 *                                                                    *
 *    Author: Marcello Bonfe'                                         *
 *                                                                    *
 *    Filename:       Timers.c                                        *
 *    Date:           20/04/2011                                      *
 *    File Version:   0.2                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the initialization and ISR for any timer.
 *
 **********************************************************************/
 
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "Timers.h"
#include "Controls.h"
#include "Comms.h"

uint8_t speed_loop_count = 0;
uint8_t overflow1_timer2, overflow2_timer2;

/******************************************************
* TIMER INIT ROUTINES
******************************************************/

// TIMER1 used for Motor3 position feedback (UPCount)
void Timer1_Init(void)
{
    // T1CON
        // Bit 15 - 1=Timer 1 is on,o is off
        // Bit 14 - Not Used
        // Bit 13 - 0=continues in idle,1=discontinues
        // Bits12-7 -     Not Used
        // Bit6 - 1=timer in gate mode using T1CK
        // Bits5-4 = clk prescalar
        // 11 = 1:256
        // 10 = 1:64
        // 01 = 1:8
        // 00 = 1:1
        // Bit 3 - Not Used
        // Bit 2 - 1= Synch external clock input
        // Bit 1 - 1=clk is rising edge of T1CK,0 = Tcy
        // Bit0 - Not Used
    T1CON = 0x0006;  //Timer1 settato prescale 1:1,16 bit, Fcy
       
    TMR1=0;
    
    // RESET INTERRUPT FLAG 
    IFS0bits.T1IF = 0;   
      
    // INTERRUPT DISABLE
    IEC0bits.T1IE = 0;
      
    //Set the interrupt priority
    //7 = maximum
    //4 = default
    //0 = disable int.
    //IPC0bits.T1IP = 6;  
      
    //ENABLE TIMER
    T1CONbits.TON=1;     
}


// TIMER4 used for Motor3 position feedback (DOWNCount)
void Timer4_Init(void)
{
    // T4CON
        // Bit 15 - 1=Timer 1 is on,o is off
        // Bit 14 - Not Used
        // Bit 13 - 0=continues in idle,1=discontinues
        // Bits12-7 -     Not Used
        // Bit6 - 1=timer in gate mode using T4CK
        // Bits5-4 = clk prescalar
        // 11 = 1:256
        // 10 = 1:64
        // 01 = 1:8
        // 00 = 1:1
        // Bit 3 - T45 - 1=Timer4/5 as 32 bit timer
        // Bit 2 - Not used
        // Bit 1 - 1=clk is rising edge of T1CK,0 = Tcy
        // Bit0 - Not Used
    T4CON = 0x0002;     //Timer Input Clock Prescale Select bits 1:1
    
    TMR4=0;
    
    // RESET INTERRUPT FLAG 
    IFS1bits.T4IF = 0;   
      
    // INTERRUPT DISABLE 
    IEC1bits.T4IE = 0;
      
    //Set the interrupt priority
    //7 = maximum
    //4 = default
    //0 = disable int.
    //IPC6bits.T4IP = 5;  
      
    //ENABLE TIMER
    T4CONbits.TON=1;     
}

//TIMER2 IS USED FOR INTERRUPT CAPTURE
void Timer2_Init(void)
{
	TMR2 = 0;

	//Reset interrupt flag
	IFS0bits.T2IF = 0;
	//set priotity (?)

	//enable interrupt
	IEC0bits.T2IE = 1;
	//enable timer
	T2CONbits.TON = 1;
	
	//PR2 = 0xFFFF;

	overflow1_timer2 = 0;
	overflow2_timer2 = 0;
}

//TIMER5 used to schedule high-freq control loop
void Timer5_Init(void)
{
    // T5CON
        // Bit 15 - 1=Timer 1 is on,o is off
        // Bit 14 - Not Used
        // Bit 13 - 0=continues in idle,1=discontinues
        // Bits12-7 -     Not Used
        // Bit6 - 1=timer in gate mode using T1CK
        // Bits5-4 = clk prescalar
        // 11 = 1:256
        // 10 = 1:64
        // 01 = 1:8
        // 00 = 1:1
        // Bit 3 - Not used
        // Bit 2 - Not used
        // Bit 1 - 1=clk is rising edge of T1CK,0 = Tcy
        // Bit0 - Not Used
    T5CON = 0x0010;    // Timer set w/1:8 prescaler
    
    // CHECK THAT FCY/N IS CONSISTENT WITH 1:N PRESCALER!!!
    PR5 = (FCY/8) / POS_LOOP_FREQ;
      
    // RESET INTERRUPT FLAG 
    IFS1bits.T5IF = 0;   
      
    // INTERRUPT ENABLE 
    IEC1bits.T5IE = 1;
      
    //Set the interrupt priority
    //7 = maximum
    //4 = default
    //0 = disable int.
    IPC7bits.T5IP = 5;  
      
    //ENABLE TIMER
    T5CONbits.TON=1;     
}

//TIMER 2 INTERRUPT SERVICE ROUTINE
void __attribute__((interrupt,auto_psv)) _T2Interrupt(void)
{
	IFS0bits.T2IF = 0; //reset interrupt flag

	overflow1_timer2 = 1;
	overflow2_timer2 = 1;
}


/************************************************************
* _T5Interrupt() is the Timer5 interrupt service routine (ISR).
* The routine must have global scope in order to be an ISR.
* The ISR name is chosen from the device linker script.
************************************************************/
void __attribute__((interrupt,no_auto_psv)) _T5Interrupt(void) 
{
    IFS1bits.T5IF = 0; //Clear Timer5 Interrupt Flag
    
    UpdateEncoder1();
    UpdateEncoder2();
    UpdateEncoder3();

    if(control_flags.pos_loop_active)
    {
        PositionLoops();
    }
    else if(control_flags.track_loop_active)
    {
        TrackingLoops();
    }
}
