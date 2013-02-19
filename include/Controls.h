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
 *                                                                    *
 *    Author: Marcello Bonfe'                                         *
 *                                                                    *
 *    Filename:       Controls.h                                         *
 *    Date:           20/04/2011                                      *
 *    File Version:   0.2                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  Header file for the control loops functions.
 *
 **********************************************************************/

#ifndef CONTROLS_H
#define CONTROLS_H

//definizione di una struttura tHome
typedef struct {
    int16_t         Velocity; //velocità di homing
    int32_t         position[N_MOTOR];
    //int32_t         position1; //positione di homing del motore 1
    //int32_t         position2; //posizione di homing del motore 2
    //int32_t         position3; //posizione di homing del motore 3
	int16_t			offsetPosition; //differenza tra la posizione di homing e la posizione effettiva dei motori
    } tHome;

typedef struct {
	unsigned homing_active	: 1;
	unsigned state			: 3;
	unsigned done			: 1;
	//unsigned wrong		: 1;
} tHomeflags;

void UpdateEncoder(void);

void CurrentLoops(void);
void PositionLoops(void);
void TrackingLoops(void);

void homing_manager(void);

void update_delta_EE(void);
void update_delta_joints(void);

#endif
