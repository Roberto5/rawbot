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
 *    Filename:       traps.c                                         *
 *    Date:           02/08/2011                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the motion activation.
 *
 **********************************************************************/

#include "generic_defs.h"
#include "extern_globals.h"
#include "Controls.h"
#include "Trajectories.h" //for TRAJ parm/flags data-types
#include "geometry.h"
#include "Convert.h"

//float deg_to_ticks 	= encoder_ticks / 360.0;
float decdeg_to_ticks;
  
void move(delta_joints angleJoints)
{
	if((control_mode.state == AX_POS_MODE)||(control_mode.state == CART_MODE))
	{
		if(!TRAJMotor1_f.exec && !TRAJMotor2_f.exec && !TRAJMotor3_f.exec)		//if exec=1 it doesn't execute any command
		{
		    TRAJMotor1.qdPosCOM = convert_rad_to_decdeg(angleJoints.theta1) * decdeg_to_ticks;
	        TRAJMotor2.qdPosCOM = convert_rad_to_decdeg(angleJoints.theta2) * decdeg_to_ticks;
	        TRAJMotor3.qdPosCOM = convert_rad_to_decdeg(angleJoints.theta3) * decdeg_to_ticks;
	
			TRAJMotor1_f.exec = 1;
	    	TRAJMotor2_f.exec = 1;
	    	TRAJMotor3_f.exec = 1;
		}
	}
	else if (control_mode.state == TRACK_MODE)
	{
		Joint1NLFStatus.qdRcommand = (int32_t)( (float) convert_rad_to_decdeg(angleJoints.theta1) * decdeg_to_ticks);
        Joint2NLFStatus.qdRcommand = (int32_t)( (float) convert_rad_to_decdeg(angleJoints.theta2) * decdeg_to_ticks);
        Joint3NLFStatus.qdRcommand = (int32_t)( (float) convert_rad_to_decdeg(angleJoints.theta3) * decdeg_to_ticks);
	}
}
