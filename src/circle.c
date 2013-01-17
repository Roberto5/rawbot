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
 *    Filename:       circle.c                                        *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the circle routine.
 *
 **********************************************************************/

#include "sys_hw.h"
#include "generic_defs.h"
#include "extern_globals.h"
#include "Comms.h"
#include "lib_crc.h"
#include "Trajectories.h" //for TRAJ parm/flags data-types
#include "geometry.h"
#include "circle.h" 
#include "SACT_Protocol.h"
#include "Kinematix.h"
#include "math.h"
#include "geometry.h"
#include "motion.h"

int16_t x_pos, y_pos;
int status_circle, move_count;

int circle(int16_t x_centre, int16_t y_centre, int16_t z_centre, int16_t radius)
{
	status_circle = delta_calcInverse(x_centre, y_centre, z_centre);
	move_count = 0;
	
	switch(status_circle)		//centro cerchio
	{
		case -1://point out of space
			return -1;
			break;
			
		case -3://danger joints
			return -3;
			break;	
	}

	while(move_count < 36)
	{
		if(!TRAJMotor1_f.exec && !TRAJMotor2_f.exec && !TRAJMotor3_f.exec)
		{
			x_pos = radius * cosf(move_count * 2 * pi / 36);
			y_pos = radius * sinf(move_count * 2 * pi / 36);
		
			status_circle = delta_calcInverse(x_pos, y_pos, z_centre);
	
			switch(status_circle)
			{
				case -1://point out of space
					return -1;
					break;
			
				case -3://danger joints
					return -3;
					break;	
			}

			move_count++;

		}
	}

	return 0;
}



