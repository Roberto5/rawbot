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
 *    Filename:       InputCapture.h                                       *
 *    Date:           15/05/2012                                      *
 *    File Version:   1.3                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  Header file for Input capture module.
 *
 **********************************************************************/

#ifndef INPUTCAPTURE_H
#define INPUTCAPTURE_H

//INPUT CAPTURE FOR TIMER SELECT BIT
#define	IC_TIMER2	1
#define IC_TIMER3	0

//SELECT NUMBER OF CAPTURE FOR INTERRUPT BIT
#define	IC_INT_1CAPTURE	0
#define	IC_INT_2CAPTURE	1
#define	IC_INT_3CAPTURE	2
#define	IC_INT_4CAPTURE	3

//INPUT CAPTURE FOR MODE SELECT BIT
#define	IC_TURNED_OFF	0
#define	IC_EVERY_RFEDGE	1 // every rising and falling edge
#define	IC_EVERY_FEDGE	2 // every falling edge
#define	IC_EVERY_REDGE	3 // every rising edge
#define	IC_EVERY_4EDGE	4 // every 4 rising edge
#define	IC_EVERY_16EDGE	5 // every 16 rising edge
#define	UNUSED			6

//INPUT CAPTURE FIRST USE
extern uint8_t IC1_FIRST;
extern uint8_t IC2_FIRST;
//extern uint8_t IC7_FIRST;

//PERIOD AND PULSE VARIABLE FOR INPUT CAPTURE
extern int32_t IC1Period, IC2Period, IC7Period;
extern int16_t IC1Pulse, IC2Pulse, IC7Pulse;

//extern uint16_t IC1currentPeriod_temp, IC1previousPeriod_temp;
//extern uint16_t IC2currentPeriod_temp, IC2previousPeriod_temp;

//Functions with Global Scope
void IC1_Init(void);
void IC2_Init(void);
//void IC7_Init(void);

void __attribute__((interrupt,auto_psv)) _IC1Interrupt(void);
void __attribute__((interrupt,auto_psv)) _IC2Interrupt(void);
//void __attribute__((interrupt,auto_psv)) _IC7Interrupt(void);

#endif
