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
 *    Author: Marco Altafini                                         *
 *                                                                    *
 *    Filename:       InputCapture.c                                        *
 *    Date:           15/05/2012                                      *
 *    File Version:   1.3                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the initialization and ISR for input capture module.
 *
 **********************************************************************/

#include "sys_hw.h"
#include "extern_globals.h"
#include "Timers.h"
#include "InputCapture.h"

uint8_t IC1_FIRST;
uint8_t IC2_FIRST;
//uint8_t IC7_FIRST;

int32_t IC1Period;
int32_t IC2Period;
int32_t IC7Period;

int16_t IC1Pulse;
int16_t IC2Pulse;
int16_t IC7Pulse;

//local variablle
uint16_t IC1currentPeriod, IC1previousPeriod;
uint16_t IC2currentPeriod, IC2previousPeriod;
//uint16_t IC1currentPeriod_temp, IC1previousPeriod_temp;
//uint16_t IC2currentPeriod_temp, IC2previousPeriod_temp;
//int16_t IC7currentPeriod, IC7previousPeriod;

//INPUT CAPTURE INIT ROUTINE
void IC1_Init(void)
{
	IC1CONbits.ICTMR = IC_TIMER2;
	IC1CONbits.ICI = IC_INT_1CAPTURE;
	IC1CONbits.ICM = IC_EVERY_REDGE;

	//reset interrupt flag
	IFS0bits.IC1IF = 0;
	//enable interrupt
	IEC0bits.IC1IE = 1;
	//set priority level (?)
	//IPC0bits.IC1IP = 4;
	//first use
	IC1_FIRST = 1;
	IC1Period = 0;
	IC1Pulse = 0;
}

void IC2_Init(void)
{
	IC2CONbits.ICTMR = IC_TIMER2;
	IC2CONbits.ICI = IC_INT_1CAPTURE;
	IC2CONbits.ICM = IC_EVERY_REDGE;

	//reset interrupt flag
	IFS0bits.IC2IF = 0;
	//enable interrupt
	IEC0bits.IC2IE = 1;
	//set priority level (?)
	//IPC0bits.IC2IP = 4;
	//first use
	IC2_FIRST = 1;
	IC2Period = 0;
	IC2Pulse = 0;
}

/*void IC7_Init(void)
{
	IC7CONbits.ICTMR = IC_TIMER2;
	IC7CONbits.ICI = IC_INT_1CAPTURE;
	IC7CONbits.ICM = IC_EVERY_REDGE;

	//reset interrupt flag
	IFS1bits.IC7IF = 0;
	//enable interrupt
	IEC1bits.IC7IE = 1;
	//set priority level (?)
	//IPC0bits.IC2IP = 4;
	//first use
	IC7_FIRST = 1;
	IC7Period = 0;
	IC7Pulse = 0;
}*/

//INPUT CAPTURE INTERRUPT SERVICE ROUTINE
void __attribute__((interrupt,auto_psv)) _IC1Interrupt(void)
{
	IFS0bits.IC1IF = 0; //reset interrupt flag

	if (!IC1_FIRST)
	{
		IC1currentPeriod = IC1BUF;
		if (!overflow1_timer2)
			IC1Period += (IC1currentPeriod - IC1previousPeriod);
		else
		{
			IC1Period += (IC1currentPeriod + (0xFFFF - IC1previousPeriod));
			overflow1_timer2 = 0;
		} 
//		IC1previousPeriod_temp = IC1previousPeriod;
//		IC1currentPeriod_temp = IC1currentPeriod;
		IC1previousPeriod = IC1currentPeriod;
	}
	else
	{
		IC1previousPeriod = IC1BUF;
		IC1Period = IC1previousPeriod;
		IC1_FIRST = 0;
	}
	
	if (!QEI1CONbits.UPDN)	
	{
		IC1Pulse ++;
	}
	else
	{
		IC1Pulse --;
	}
}


void __attribute__((interrupt,auto_psv)) _IC2Interrupt(void)
{
	IFS0bits.IC2IF = 0; //reset interrupt flag

	if (!IC2_FIRST)
	{
		IC2currentPeriod = IC2BUF;
		if (!overflow2_timer2)
			IC2Period += (IC2currentPeriod - IC2previousPeriod);//periodo tra l'interrupt precedente e quello corrente
		else
		{
			IC2Period += (IC2currentPeriod + (0XFFFF - IC2previousPeriod));
			overflow2_timer2 = 0;
		} 
//		IC2previousPeriod_temp = IC2previousPeriod;
//		IC2currentPeriod_temp = IC2currentPeriod;
		IC2previousPeriod = IC2currentPeriod;
	}
	else
	{
		IC2previousPeriod = IC2BUF;
		IC2Period = IC2BUF;
		IC2_FIRST = 0;
	}
	
	if (!QEI2CONbits.UPDN)
	{
		IC2Pulse ++;
	}
	else
	{
		IC2Pulse --;
	}
}

/*void __attribute__((interrupt,auto_psv)) _IC7Interrupt(void)
{
	IFS1bits.IC7IF = 0; //reset interrupt flag

	if (!IC7_FIRST)
	{
		IC7currentPeriod = IC7BUF;
		if (!overflow3_timer2)
			IC7Period += (IC7currentPeriod - IC7previousPeriod);
		else
		{
			IC7Period += (IC7currentPeriod + (PR2 - IC7previousPeriod));
			overflow3_timer2 = 0;
		} 
		IC7previousPeriod = IC7currentPeriod;
	}
	else
	{
		IC7previousPeriod = IC7BUF;
		IC7_FIRST = 0;
	}
	IC7Pulse++;
}*/

