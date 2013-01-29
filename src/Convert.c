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
 *    Author: Marco Altafini                                         *
 *                                                                    *
 *    Filename:       Convert.c                                     *
 *    Date:           06/06/2012                                      *
 *    File Version:   1.3                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the functions for conversion operation.
 *
 **********************************************************************/
#include "geometry.h"
float decdeg_to_rad = (0.1 * pi/180.0); //conversion: decimal degrees-->radians
float deg_to_rad = pi/180.0; //conversion: decimal degrees-->radians
float rad_to_deg = 180.0 / pi ; //conversion: radians-->degrees
float rad_to_decdeg = 1800.0/pi; //conversion: radians-->decimal degrees

float convert_decdeg_to_rad(float th_decdeg)
{
	return th_decdeg * decdeg_to_rad; //return angle in radians
}
float convert_deg_to_rad(float th_deg)
{
	return th_deg * deg_to_rad; //return angle in radians
}
float convert_rad_to_deg(float th_rad)
{
	return th_rad * rad_to_deg; //return angle in degrees
}
float convert_rad_to_decdeg(float th_rad)
{	
	return th_rad * rad_to_decdeg; //return angle in decimal degrees
}
//convert decimal millimeters in meters
float convert_decmill_to_meters(float x_decmill)
{
	return x_decmill / 10000;
}
//convert meters in decimal millimeters
float convert_meters_to_decmill(float x_meters)
{
	return x_meters * 10000;
}
