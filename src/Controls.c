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
 *    Filename:       Controls.c                                      *
 *    Date:           20/04/2011                                      *
 *    File Version:   0.2                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 ***********************************************************************
 *    Code Description
 *  
 *  This file contains the control loops functions.
 *
 **********************************************************************/

#include "sys_hw.h"
#include "extern_globals.h"
#include "Kinematix.h"
#include "Controls.h"
#include "Timers.h"
#include "PID.h"
#include "PWM.h"
#include "ADC_DMA.h" // for mcurrentXsamp buffers..
#include "geometry.h"
#include "motion.h"
#include "my_fractmath.h" // for all math stuffs
#include "InputCapture.h"
#include "Convert.h"

/********************************************
 * GLOBAL VARIABLES
 *******************************************/
// PID parameters and flags structures
tPIDParm PIDCurrent1;
tPIDParm PIDCurrent2;
tPIDParm PIDCurrent3;
tPIDParm PIDPos1;
tPIDParm PIDPos2;
tPIDParm PIDPos3;

tPIDflags PIDCurrent1_f;
tPIDflags PIDCurrent2_f;
tPIDflags PIDCurrent3_f;
tPIDflags PIDPos1_f;
tPIDflags PIDPos2_f;
tPIDflags PIDPos3_f;

// TRAJ parameters and flags structures
tTRAJParm TRAJMotor1;
tTRAJParm TRAJMotor2;
tTRAJParm TRAJMotor3;

tTRAJflags TRAJMotor1_f;
tTRAJflags TRAJMotor2_f;
tTRAJflags TRAJMotor3_f;

// nonlinear filter smoothing
tNLFStatus Joint1NLFStatus;
tNLFStatus Joint2NLFStatus;
tNLFStatus Joint3NLFStatus;

tNLFOut Joint1NLFOut;
tNLFOut Joint2NLFOut;
tNLFOut Joint3NLFOut;

//Homing
tHome home;

tHomeflags home_f;

// limits
uint32_t NLF_vel_max;
uint32_t NLF_acc_max_shift;

//temporary variable for input capture
volatile int32_t IC1PeriodTmp, IC2PeriodTmp;
volatile int16_t IC1PulseTmp, IC2PulseTmp;

/************************************************
 * LOCAL FUNCTIONS
 ***********************************************/
void UpdateEncoder1(void);
void UpdateEncoder2(void);
void UpdateEncoder3(void);

void homing_manager(void);
/***********************************************
 * LOCAL VARIABLES
 ***********************************************/
// Temps for odometry calc.
int16_t PosCount1,PosCount2,UpCount,DownCount;

// TO MANAGE LMD18200 SIGN INVERSION
#define BLANKS 2
uint8_t DIR1_PREV, DIR2_PREV,DIR3_PREV;
uint8_t DIR1_TMP, DIR2_TMP, DIR3_TMP;
uint8_t DIR1_blank_count,DIR2_blank_count,DIR3_blank_count;
uint8_t tempidx;

/*************************************
 * Current control loops
 * for all motors
 *************************************/
void CurrentLoops(void)
{
// FIRST MOTOR
    // MANAGE SIGN OF REFERENCE (sign/magnitude control of LMD18200)
    if(rcurrent1 < 0)
    {
        PIDCurrent1.qdInRef  = (int32_t)(-rcurrent1);
        DIR1_TMP = ~direction_flags.motor1_dir;
    }
    else
    {    
        PIDCurrent1.qdInRef  = (int32_t)rcurrent1;
        DIR1_TMP = direction_flags.motor1_dir;
    }
    
    // Manage sign inversion
    if(DIR1_PREV != DIR1_TMP)
    {
        InitPID(&PIDCurrent1, &PIDCurrent1_f,-1);
        for(tempidx=0; tempidx < MCURR_MAV_ORDER; tempidx++)
            mcurrent1samp[tempidx] = 0;
        DIR1_blank_count = BLANKS;
    }    

    if(DIR1_blank_count == 0)
    {
        //PIDCurrent1.qdInMeas = (int32_t)(mcurrent1_filt - mcurrent1_offset);
        PIDCurrent1.qdInMeas = (int32_t)(mcurrent1 - mcurrent1_offset);
    
        CalcPI(&PIDCurrent1, &PIDCurrent1_f);
    }
    else
        DIR1_blank_count--;
    
    DIR1_PREV = DIR1_TMP;
      
// SECOND MOTOR
    // MANAGE SIGN OF REFERENCE (sign/magnitude control of LMD18200)
    if(rcurrent2 < 0)
    {
        PIDCurrent2.qdInRef  = (int32_t)(-rcurrent2);
        DIR2_TMP = ~direction_flags.motor2_dir;
    }
    else
    {    
        PIDCurrent2.qdInRef  = (int32_t)rcurrent2;
        DIR2_TMP = direction_flags.motor2_dir;
    }
    
    // Manage sign inversion
    if(DIR2_PREV != DIR2_TMP)
    {
        InitPID(&PIDCurrent2, &PIDCurrent2_f,-1);
        for(tempidx=0; tempidx < MCURR_MAV_ORDER; tempidx++)
            mcurrent2samp[tempidx] = 0;
        DIR2_blank_count = BLANKS;
    }
    
    if(DIR2_blank_count == 0)
    {
        //PIDCurrent2.qdInMeas = (int32_t)(mcurrent2_filt - mcurrent2_offset);
        PIDCurrent2.qdInMeas = (int32_t)(mcurrent2 - mcurrent2_offset);
    
        CalcPI(&PIDCurrent2, &PIDCurrent2_f);
    }
    else
        DIR2_blank_count--;
    
    DIR2_PREV = DIR2_TMP;

// THIRD MOTOR
    // MANAGE SIGN OF REFERENCE (sign/magnitude control of LMD18200)
    if(rcurrent3 < 0)
    {
        PIDCurrent3.qdInRef  = (int32_t)(-rcurrent3);
        DIR3_TMP = ~direction_flags.motor3_dir;
    }
    else
    {    
        PIDCurrent3.qdInRef  = (int32_t)rcurrent3;
        DIR3_TMP = direction_flags.motor3_dir;
    }
    
    // Manage sign inversion
    if(DIR3_PREV != DIR3_TMP)
    {
        InitPID(&PIDCurrent3, &PIDCurrent3_f,-1);
        for(tempidx=0; tempidx < MCURR_MAV_ORDER; tempidx++)
            mcurrent3samp[tempidx] = 0;
        DIR3_blank_count = BLANKS;
    }
    
    if(DIR3_blank_count == 0)
    {
        //PIDCurrent3.qdInMeas = (int32_t)(mcurrent3_filt - mcurrent3_offset);
        PIDCurrent3.qdInMeas = (int32_t)(mcurrent3 - mcurrent3_offset);
    
        CalcPI(&PIDCurrent3, &PIDCurrent3_f);
    }
    else
        DIR3_blank_count--;
    
    DIR3_PREV = DIR3_TMP;
      
// IMPORTANT: INVERTED FIRING!!
    DIR1 = DIR1_TMP;
    DIR2 = DIR2_TMP;
    DIR3 = DIR3_TMP;
    P1DC1 = FULL_DUTY - (PIDCurrent1.qOut + ZERO_DUTY);
    P1DC2 = FULL_DUTY - (PIDCurrent2.qOut + ZERO_DUTY);
    P2DC1 = FULL_DUTY - (PIDCurrent3.qOut + ZERO_DUTY);
}


/*************************************
 * Position control loop
 *************************************/
void PositionLoops(void)
{
	if (home_f.homing_active)
	{
		homing_manager();
	}
	else
	{
		// MOTOR1 POSITION CONTROL
	    if(TRAJMotor1_f.enable && !TRAJMotor1_f.active)			//if enabled but not active
	        TRAJMotor1.qdPosition = mposition1;
	    
	    PosTRAJ(&TRAJMotor1, &TRAJMotor1_f);					//trapezoidal motion
	
	  /*  if(TRAJMotor1_f.done)			//motion completed, back exec=0
	       TRAJMotor1_f.exec = 0;*/
	    
	// MOTOR2 POSITION CONTROL
	    if(TRAJMotor2_f.enable && !TRAJMotor2_f.active)
	        TRAJMotor2.qdPosition = mposition2;
	    
	    PosTRAJ(&TRAJMotor2, &TRAJMotor2_f);
	
	   /* if(TRAJMotor2_f.done)
	        TRAJMotor2_f.exec = 0;*/
	
	// MOTOR3 POSITION CONTROL
	    if(TRAJMotor3_f.enable && !TRAJMotor3_f.active)
	        TRAJMotor3.qdPosition = mposition3;
	    
	    PosTRAJ(&TRAJMotor3, &TRAJMotor3_f);
	
	   /* if(TRAJMotor3_f.done)
	        TRAJMotor3_f.exec = 0;*/
	}
		//delta_calcForward(theta1,theta2,theta3);
		
	    PIDPos1.qdInMeas = mposition1;
	    PIDPos1.qdInRef  = TRAJMotor1.qdPosition; 
	    CalcPID(&PIDPos1, &PIDPos1_f);
	    
	    rcurrent1 = PIDPos1.qOut;

	    PIDPos2.qdInMeas = mposition2;
	    PIDPos2.qdInRef  = TRAJMotor2.qdPosition;
	    CalcPID(&PIDPos2, &PIDPos2_f);
	    
	    rcurrent2 = PIDPos2.qOut;

		PIDPos3.qdInMeas = mposition3;
	    PIDPos3.qdInRef  = TRAJMotor3.qdPosition;
	    CalcPID(&PIDPos3, &PIDPos3_f);
	    
	    rcurrent3 = PIDPos3.qOut;
	
#ifdef DEVELOP_MODE
#ifdef LOG_POSLOOP
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        dataLOG1[dataLOGIdx] = PIDPos1.qdInRef;
        dataLOG2[dataLOGIdx] = mposition1;
        //dataLOG3[dataLOGIdx] = (int16_t)PIDPos2.qdInRef;
        //dataLOG4[dataLOGIdx] = 0;
        
        dataLOGIdx++;
        if(dataLOGIdx == MAXLOG) dataLOGIdx = 0;
        
        dataLOGdecim = 0;
    }// IF DECIMATION    
    
    
#endif //LOG_POSLOOP
#endif //DEVELOP_MODE

}// END ANGLE/POS LOOPS

/*************************************
 * Tracking control loop with Nonlinear
 * Smoothing Filter
 *************************************/
void TrackingLoops(void)
{
// MOTOR1 POSITION CONTROL
     
    NLFilter2Fx(&Joint1NLFOut, &Joint1NLFStatus, NLF_vel_max, NLF_acc_max_shift, POS_LOOP_FcSHIFT);

    PIDPos1.qdInMeas = mposition1;
    PIDPos1.qdInRef  = Joint1NLFOut.qdX; 
    CalcPID(&PIDPos1, &PIDPos1_f);
    
    rcurrent1 = PIDPos1.qOut;
    
// MOTOR2 POSITION CONTROL
    NLFilter2Fx(&Joint2NLFOut, &Joint2NLFStatus, NLF_vel_max, NLF_acc_max_shift, POS_LOOP_FcSHIFT);
    
    PIDPos2.qdInMeas = mposition2;
    PIDPos2.qdInRef  = Joint2NLFOut.qdX;
    CalcPID(&PIDPos2, &PIDPos2_f);
    
    rcurrent2 = PIDPos2.qOut;

// MOTOR3 POSITION CONTROL
    NLFilter2Fx(&Joint3NLFOut, &Joint3NLFStatus, NLF_vel_max, NLF_acc_max_shift, POS_LOOP_FcSHIFT);
    
    PIDPos3.qdInMeas = mposition3;
    PIDPos3.qdInRef  = Joint3NLFOut.qdX;
    CalcPID(&PIDPos3, &PIDPos3_f);
    
    rcurrent3 = PIDPos3.qOut;

#ifdef DEVELOP_MODE
#ifdef LOG_TRACKLOOP
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        dataLOG1[dataLOGIdx] = Joint1NLFOut.qdX;
        dataLOG2[dataLOGIdx] = Joint1NLFOut.qdXdot;
        //dataLOG3[dataLOGIdx] = (int16_t)PIDPos2.qdInRef;
        //dataLOG4[dataLOGIdx] = 0;
        
        dataLOGIdx++;
        if(dataLOGIdx == MAXLOG) dataLOGIdx = 0;
        
        dataLOGdecim = 0;
    }// IF DECIMATION    
    
    
#endif //LOG_POSLOOP
#endif //DEVELOP_MODE

}// END TRACKING LOOPS with Nonlinear Smoothing Filter

/*************************************
 * Functions to update encoder counts
 * for ALL motors
 *************************************/
void UpdateEncoder1(void)
{
#ifdef SIMULATE
    if(DIR1)    
        mvelocity1 -= (mcurrent1_filt - mcurrent1_offset) >> 4;
    else
        mvelocity1 += (mcurrent1_filt - mcurrent1_offset) >> 4;
    if(mvelocity1 > 550)
        mvelocity1 = 550;
    else if(mvelocity1 < -550)
        mvelocity1 = -550;
	if (((mposition1 / (encoder_ticks / 3600.0))<-200)&&(home_f.state!=0))
		mvelocity1 = 0; 
    mposition1 += (int32_t)mvelocity1;
#endif

#ifdef PROTO_BOARD
    mvelocity1 = PosCount1;
    PosCount1 = POS1CNT;
    mvelocity1 -= PosCount1;

	IC1PulseTmp = IC1Pulse;
	IC1PeriodTmp = IC1Period;
	IC1Pulse = 0;
	IC1Period = 0;
    
	if (IC1PulseTmp != 0)
	{
		velocity1RPM = (int16_t)(((int32_t)(kvel*IC1PulseTmp))/IC1PeriodTmp);
		//velocity1RPM_temp = (int16_t)(((int32_t)(kvel*IC1Pulse))/((IC1currentPeriod_temp+IC1previousPeriod_temp)/2));
	}
	else
		velocity1RPM = 0;

    mposition1 += (int32_t)mvelocity1;
#endif

//mtheta1 = mposition1 / (encoder_ticks / 3600.0);
//mtheta1 = RSH((mposition1 * decdeg_to_ticks_int),10); // deg to ticks is 5.10 fixed-point so RightSHift!
}

void UpdateEncoder2(void)
{
#ifdef SIMULATE
	if (home_f.state!=2)
	{
    if(DIR2)    
        mvelocity2 -= (mcurrent2_filt - mcurrent2_offset) >> 4;
    else
        mvelocity2 += (mcurrent2_filt - mcurrent2_offset) >> 4;
    if(mvelocity2 > 550)
        mvelocity2 = 550;
    else if(mvelocity2 < -550)
        mvelocity2 = -550;
	if (((mposition2 / (encoder_ticks / 3600.0))<-200)&&(home_f.state!=0))
		mvelocity2 = 0;
    mposition2 += (int32_t)mvelocity2;
	}
#endif

#ifdef PROTO_BOARD
    mvelocity2 = PosCount2;
    PosCount2 = POS2CNT;
    mvelocity2 -= PosCount2;
	
	IC2PulseTmp = IC2Pulse;
	IC2PeriodTmp = IC2Period;
	IC2Pulse = 0;
	IC2Period = 0;

	if(IC2PulseTmp != 0)
	{
		velocity2RPM = (int16_t)(((int32_t)(kvel*IC2PulseTmp))/IC2PeriodTmp);
//		velocity2RPM_temp = (int16_t)(((int32_t)(kvel*IC2PulseTmp))/((IC2currentPeriod_temp+IC2previousPeriod_temp)/2));
	}
	else
		velocity2RPM = 0;

    mposition2 += (int32_t)mvelocity2;
#endif

//mtheta2 = mposition2 / (encoder_ticks / 3600.0);
//mtheta2 = RSH((mposition2 * decdeg_to_ticks_int),10); // deg to ticks is 5.10 fixed-point so RightSHift!
}

void UpdateEncoder3(void)
{
#ifdef SIMULATE
	if ((home_f.state!=2)&&(home_f.state!=3))
{
    if(DIR3)    
        mvelocity3 -= (mcurrent3_filt - mcurrent3_offset) >> 4;
    else
        mvelocity3 += (mcurrent3_filt - mcurrent3_offset) >> 4;
    if(mvelocity3 > 550)
        mvelocity3 = 550;
    else if(mvelocity3 < -550)
        mvelocity3 = -550;
	if (((mposition3 / (encoder_ticks / 3600.0))<-200)&&(home_f.state!=0))
		mvelocity3 = 0;
    mposition3 += (int32_t)mvelocity3;
}
#endif

#ifdef PROTO_BOARD
    mvelocity3 = DownCount;
    mvelocity3 -=UpCount;
    if(direction_flags.encoder3_chB_lead)
    {
        DownCount=TMR4;
        UpCount=TMR1;
    }
    else
    {
        DownCount=TMR1;
        UpCount=TMR4;
    }
    mvelocity3+=UpCount;    
    mvelocity3-=DownCount;
      
    mposition3+=(int32_t)mvelocity3; 
#endif

//mtheta3 = mposition3 / (encoder_ticks / 3600.0);
//mtheta3 = RSH((mposition3 * decdeg_to_ticks_int),10); // deg to ticks is 5.10 fixed-point so RightSHift!
}

void homing_manager(void)
{
	
	switch (home_f.state)
	{
		case 0:
			angleJoints_temp.theta1 = 0;
			angleJoints_temp.theta2 = 0;
			angleJoints_temp.theta3 = 0;
		
			move(angleJoints_temp);
		
			home_f.state = 1;
		break;

		case 1:
			if(TRAJMotor1_f.enable && !TRAJMotor1_f.active)
				TRAJMotor1.qdPosition = mposition1;
			PosTRAJ(&TRAJMotor1, &TRAJMotor1_f);

			if(TRAJMotor2_f.enable && !TRAJMotor2_f.active)
				TRAJMotor2.qdPosition = mposition2;
			PosTRAJ(&TRAJMotor2, &TRAJMotor2_f);

			if(TRAJMotor3_f.enable && !TRAJMotor3_f.active)
				TRAJMotor3.qdPosition = mposition3;
			PosTRAJ(&TRAJMotor3, &TRAJMotor3_f);

			if((!TRAJMotor1_f.exec)&&(!TRAJMotor2_f.exec)&&(!TRAJMotor3_f.exec))
			{	
				//home.position1 = mposition1;
				TRAJMotor1.qdPosition = mposition1;
				TRAJMotor1.qVelCOM = -home.Velocity;
				
				//home.position2 = mposition2;
				TRAJMotor2.qdPosition = mposition2;
				TRAJMotor2.qVelCOM = -home.Velocity;

				//home.position3 = mposition3;
				TRAJMotor3.qdPosition = mposition3;
				TRAJMotor3.qVelCOM = -home.Velocity;

				home_f.state = 2;
			}	
			break;
			
		case 2: //homing motor1
			JogTRAJ(&TRAJMotor1, &TRAJMotor1_f); //JOG motion	
			//home.position1 = TRAJMotor1.qdPosition;
					
			if ((MyAbs(TRAJMotor1.qdPosition - mposition1)) > 1000)
			{
				home_f.state = 3;
                mposition1 = home.position1;
				TRAJMotor1.qdPosition = home.position1 + 100;
			}
			break;

		case 3:	//homing motor2
			JogTRAJ(&TRAJMotor2, &TRAJMotor2_f);//JOG motion
			//home.position2 = TRAJMotor2.qdPosition;
	
			if ((MyAbs(TRAJMotor2.qdPosition - mposition2)) > 1000)
			{
				home_f.state = 4;
                mposition2 = home.position2;
				TRAJMotor2.qdPosition = home.position2 + 100;
			}
			break;

		case 4: //homing motor3
			JogTRAJ(&TRAJMotor3, &TRAJMotor3_f); //JOG motion
			//home.position3 = TRAJMotor3.qdPosition;
		
			if ((MyAbs(TRAJMotor3.qdPosition - mposition3)) > 1000)
			{
				home_f.state = 5;
                mposition3 = home.position3;
				TRAJMotor3.qdPosition = home.position3 + 100;
			}
			break;
			
			case 5:
				home_f.done = 1;
				home_f.homing_active = 0;
			
				//theta1 = RSH((home.position1 * decdeg_to_ticks_int),8); //-200;
				//theta2 = RSH((home.position2 * decdeg_to_ticks_int),8);//-200;
				//theta3 = RSH((home.position3 * decdeg_to_ticks_int),8);//-200;

				//delta_calcForward(&angleJoints_temp);
			break;

			default: break;
	}
}

void update_delta_joints(void)
{
	float ticks_to_deg = 360.0 / encoder_ticks;

	angleJoints_actual.theta1 = convert_deg_to_rad(mposition1 * ticks_to_deg);
	angleJoints_actual.theta2 = convert_deg_to_rad(mposition2 * ticks_to_deg);
	angleJoints_actual.theta3 = convert_deg_to_rad(mposition3 * ticks_to_deg);
}

void update_delta_EE(void)
{
	delta_calcForward(&angleJoints_actual, &coordinates_actual);
}

