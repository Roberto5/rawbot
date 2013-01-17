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
 *    Author: Andrea Lazzarin                                         *
 *                                                                    *
 *    Filename:       Kinematix.c                                     *
 *    Date:           10/06/2011                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the kinematics functions.
 *
 **********************************************************************/

#include "extern_globals.h"
#include "generic_defs.h"
#include "geometry.h"
#include "PWM.h"
#include "motion.h"
#include "Kinematix.h"
#include "Comms.h"
#include "Convert.h"

#include <math.h>

/**********************************************************************/		
 
float zero = -4.428675e-015;		//zero for floating point operations (loss precision)

/**********************************************************************/
 // forward kinematics
 // theta1, theta2, theta3 measured in radiants
 int delta_calcForward(delta_joints *pJoint, delta_EE *pCoord) 
{	 
	float t = (f - e);// * tan30/2;
 
	 // calculating discriminant
	 //Coordinate dei giunti sferici
     float y1 = -(t + lf * cosf(pJoint->theta1));
     float z1 = -lf * sinf(pJoint->theta1);
 
     float y2 = (t + lf * cosf(pJoint->theta2))*sin30;
     float x2 = y2 * tan60;
     float z2 = -lf * sinf(pJoint->theta2);
 
     float y3 = (t + lf * cosf(pJoint->theta3))*sin30;
     float x3 = -y3 * tan60;
     float z3 = -lf * sinf(pJoint->theta3);
 	
	 //-----------------------------------------
     float dnm = (y2 - y1) * x3 - ( y3 - y1 ) * x2;
 
     float w1 = y1*y1 + z1*z1; //CIRCONFERENZA SUL PIANO YZ
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - le*le);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;

	if (d < 0)
		return -1; //point out of space
	
	pCoord->z = (-b-sqrt(d))/(2*a);
	pCoord->x = ((a1*pCoord->z) +b1)/dnm;
	pCoord->y = ((a2*pCoord->z)+ b2)/dnm;

	return 0;
}

/**********************************************************************/
 // inverse kinematics, calculates angle theta1 (for YZ-pane)
int delta_calcAngleYZ(float x_0, float y_0, float z_0, float *ptheta)
{	 
	//use dimension in decimal millimeters
	x_0 = convert_meters_to_decmill(x_0);
	y_0 = convert_meters_to_decmill(y_0);
	z_0 = convert_meters_to_decmill(z_0);

	 y_0 -= (convert_meters_to_decmill(e));//*0.50*tan30;
	 	
	 float y_1 = -(convert_meters_to_decmill(f));//*0.50*tan30;	//-((f/2)* tg(30))

	//Robot dimension in meters
	float rf = convert_meters_to_decmill(lf);
	float re = convert_meters_to_decmill(le);

     // z = a + b*y
     float a = (x_0*x_0 + y_0*y_0 + z_0*z_0 + rf*rf - re*re - y_1*y_1) / (2*z_0);
     float b = (y_1 - y_0) / z_0;

     // discriminant
     float d = -( a + b * y_1) * ( a + b * y_1) + rf * ( b * b * rf + rf); 

     if (d < 0) return -1; // non-existing point

     float yj = (y_1 - a * b - sqrtf(d)) / ( b * b + 1); // choosing outer point
     float zj = a + b * yj;

     *ptheta = 180.0 * atanf(-zj / (y_1 - yj)) / pi + ((yj > y_1)?180.0:0.0);
     
	 return 0;
 }

 // inverse kinematics
int delta_calcInverse(delta_joints *pJoint, delta_EE *pCoord ) 
{	 //x,y,z in meters

	float x0 = pCoord->x;
	float y0 = pCoord->y;
	float z0 = pCoord->z;

	float th1_deg = 0.0;
	float th2_deg = 0.0;
	float th3_deg = 0.0;

	float x_aux,y_aux;

     int8_t state = delta_calcAngleYZ(x0, y0, z0, &th1_deg);
     
	 if (state == 0) 
     {
		pJoint->theta1 = convert_deg_to_rad(th1_deg);
        x_aux = x0*cos120 + y0*sin120; // rotate coords to +120 deg
        y_aux = y0*cos120-x0*sin120;
        state = delta_calcAngleYZ(x_aux, y_aux, z0, &th2_deg);  
     }
     
	 if (state == 0) 
     {
		pJoint->theta2 = convert_deg_to_rad(th2_deg);
        x_aux = x0*cos120 - y0*sin120; // rotate coords to +120 deg
        y_aux = y0*cos120+x0*sin120;
        state = delta_calcAngleYZ(x_aux, y_aux, z0, &th3_deg);  
     }
     
	 if (state == 0)
		pJoint->theta3 = convert_deg_to_rad(th3_deg);

	return state;
 }


/**************************************************
 * CHECK if joints positions are reachable
 *************************************************/
int joints_accessible_angle(float th1, float th2, float th3)
{
    if((convert_rad_to_deg(th1) < -(negJLim))||(convert_rad_to_deg(th1) > posJLim))
        return -3;
    if((convert_rad_to_deg(th2) < -(negJLim))||(convert_rad_to_deg(th2) > posJLim))
        return -3;
    if((convert_rad_to_deg(th3) < -(negJLim))||(convert_rad_to_deg(th3) > posJLim))
        return -3;

    return 0;
}

/**************************************************
 * CHECK if cart. position is reachable
 *************************************************/
int joints_accessible_pos(float y, float y1, float y2)
{
	float re = convert_meters_to_decmill(le); //forarm lenght in decimal millimeters

    if(convert_rad_to_deg(fabsf(asinf(y/re))) > sphJLim)
        return -3;
    if(convert_rad_to_deg(fabsf(asinf(y1/re))) > sphJLim)
        return -3;
    if(convert_rad_to_deg(fabsf(asinf(y2/re))) > sphJLim)
        return -3;

    return 0;
}


