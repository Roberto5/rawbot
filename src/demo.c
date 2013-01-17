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
 *    Author: Andrea Lazzarin                                         *
 *                                                                    *
 *    Filename:       demo.c	                                      *
 *    Date:           11/08/2011                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the demos routine.
 *
 **********************************************************************/

/*#include "sys_hw.h"
#include "generic_defs.h"
#include "extern_globals.h"
#include "Comms.h"
#include "lib_crc.h"
#include "Trajectories.h" //for TRAJ parm/flags data-types
#include "geometry.h"
#include "demo.h" 
#include "SACT_Protocol.h"
#include "Kinematix.h"
#include "math.h"
#include "geometry.h"
#include "motion.h"

//move counter
int16_t move_count, demo_count;
int16_t x_pos, y_pos;

int circle(int16_t x_center, int16_t y_center, int16_t z_center, int16_t radius)
{	
/*
	switch(delta_calcInverse(x_center, y_center, z_center))		//centro cerchio
	{
		case -1://point out of space
			return -1;
			break;
			
		case -3://danger joints
			return -3;
			break;	
	}
*/
/*	move_count = 0;

	while(move_count < 72)
	{
		if(!TRAJMotor1_f.exec && !TRAJMotor2_f.exec && !TRAJMotor3_f.exec)
		{
			x_pos = radius * cosf(move_count * 2 * pi / 360);
			y_pos = radius * sinf(move_count * 2 * pi / 360);
			delta_calcInverse(x_center, y_center, z_center);
			move_count++;

		}
	}

	return 0;
}


int square(int16_t x_center, int16_t y_center, int16_t z_center, int16_t length)
{
	move_count = 0;
	demo_count = 0;

	//positions
	int16_t positions[4][3] =
	{
		{x_center + length / 2,y_center - length / 2,z_center},
		{x_center - length / 2,y_center - length / 2,z_center},
		{x_center - length / 2,y_center + length / 2,z_center},
		{x_center + length / 2,y_center + length / 2,z_center},
	};

for(demo_count=0; demo_count<50; demo_count++)
{
	while(move_count < 4)
	{
		if(!TRAJMotor1_f.exec && !TRAJMotor2_f.exec && !TRAJMotor3_f.exec)
		{		
			delta_calcInverse(positions[move_count][0], positions[move_count][1], positions[move_count][2]);
			move_count++;
		}
	}
}
	return 0;
}

int triangle(int16_t x_center, int16_t y_center, int16_t z_center, int16_t length)
{
	move_count = 0;

	//positions
	int16_t positions[3][3] =
	{
		{x_center - length / 2,y_center - length * sqrt3,z_center},
		{x_center,y_center + 2 * length * sqrt3,z_center},
		{x_center + length / 2,y_center - length *sqrt3,z_center},
	};

	while(move_count < 3)
	{
		if(!TRAJMotor1_f.exec && !TRAJMotor2_f.exec && !TRAJMotor3_f.exec)
		{		
			delta_calcInverse(positions[move_count][0], positions[move_count][1], positions[move_count][2]);

			move_count++;

		}
	}
	return 0;
}
*/