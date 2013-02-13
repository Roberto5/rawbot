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

pid PID[3];

// TRAJ parameters and flags structures

traj TRAJ[3];
/*
tTRAJParm TRAJMotor1;
tTRAJParm TRAJMotor2;
tTRAJParm TRAJMotor3;
*/
//tTRAJflags TRAJMotor1_f;
//tTRAJflags TRAJ[1].flag;
//tTRAJflags TRAJ[2].flag;

// nonlinear filter smoothing

nlf NLF[3];
/*
tNLFStatus Joint1NLFStatus;
tNLFStatus Joint2NLFStatus;
tNLFStatus Joint3NLFStatus;

tNLFOut Joint1NLFOut;
tNLFOut Joint2NLFOut;
tNLFOut Joint3NLFOut;
*/
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
int16_t PosCount1,PosCount2,UpCount,DownCount,PosCount[2];


// TO MANAGE LMD18200 SIGN INVERSION
#define BLANKS 2
uint8_t DIR1_PREV, DIR2_PREV,DIR3_PREV;
uint8_t DIR1_TMP, DIR2_TMP, DIR3_TMP;
uint8_t DIR_blank_count[3];
uint8_t tempidx;

/*************************************
 * Current control loops
 * for all motors
 *************************************/
void CurrentLoops(void)
{
    int i,duty[3];
    for (i=0;i<3;i++) {
#ifdef BRIDGE_LAP
        // MANAGE SIGN OF MEASURE (locked anti-phase control of LMD18200)
        if (MOTOR[i].rcurrent<0)
            PID[i].Current.qdInRef=-(int32_t) MOTOR[i].rcurrent;
        else
            PID[i].Current.qdInRef=(int32_t)MOTOR[i].rcurrent;
        
        //PID[i].Current.qdInMeas = (int32_t)(MOTOR[i].mcurrent);
    
        PID[i].Current.qdInMeas=(int32_t)(MOTOR[i].mcurrent_filt);

    CalcPI(&PID[i].Current, &PID[i].flag.Current);
    PID[i].Current.qOut+=ZERO_DUTY/2;//-FULL_DUTY*0.04;
    if(MOTOR[i].direction_flags.motor_dir ^ (MOTOR[i].rcurrent<0))
        duty[i] = ZERO_DUTY - PID[i].Current.qOut; // INVERTED FIRING!
    else
        duty[i] = ZERO_DUTY + PID[i].Current.qOut;
    //duty[i]=ZERO_DUTY;
#else
    //@todo implementare il rawpower
    // FIRST MOTOR
    // "Standard" bipolar PID control, no offset, since ACS714 is used
    PID[i].Current.qdInRef  = (int32_t)MOTOR[i].rcurrent;
    if(!MOTOR[i].direction_flags.motor_dir)
        PID[i].Current.qdInMeas = (int32_t)(MOTOR[i].mcurrent_filt - MOTOR[i].mcurrent_offset);
    else
        PID[i].Current.qdInMeas = -(int32_t)(MOTOR[i].mcurrent_filt - MOTOR[i].mcurrent_offset);
    //PIDCurrent1.qdInMeas = (int32_t)(mcurrent1 - mcurrent1_offset);
    CalcPI(&PID[i].Current, &PID[i].flag.Current);
    if(PID[i].Current.qOut < 0) {
        DIR1_TMP = ~MOTOR[i].direction_flags.motor_dir;
    }
    else {
        DIR1_TMP = MOTOR[i].direction_flags.motor_dir;
    }

#endif
    }
    // IMPORTANT: INVERTED FIRING!!
    
#ifdef BRIDGE_LAP
    P1DC1=duty[0];
    P1DC2=duty[1];
    P2DC1=duty[2];
#else
    DIR1 = DIR1_TMP;
    P2DC1 = FULL_DUTY - (int16_t)MyAbs16(PID[0].Current.qOut);
    /*
    DIR2 = DIR2_TMP;
    DIR3 = DIR3_TMP;
    P1DC2 = FULL_DUTY - (int16_t)MyAbs16(PID[1].Current.qOut);
    P2DC3 = FULL_DUTY - (int16_t)MyAbs16(PID[2].Current.qOut)*/
#endif
    
    
}


/*************************************
 * Position control loop
 *************************************/
void PositionLoops(void)
{
    int i;
    if (home_f.homing_active) {
        homing_manager();
    }
    else {
        for (i=0;i<3;i++) {
            if(TRAJ[i].flag.enable && !TRAJ[i].flag.active)	//if enabled but not active
                TRAJ[i].param.qdPosition = MOTOR[i].mposition;
            PosTRAJ(&TRAJ[i].param, &TRAJ[i].flag);//trapezoidal motion
            /*if(TRAJ[i].flag.done)	//motion completed, back exec=0
                TRAJ[i].flag.exec = 0;*/
        }
    }
    //delta_calcForward(theta1,theta2,theta3);
    for (i=0;i<3;i++) {
        PID[i].Pos.qdInMeas = MOTOR[i].mposition;
        PID[i].Pos.qdInRef  = TRAJ[i].param.qdPosition;
        CalcPID(&PID[i].Pos, &PID[i].flag.Pos);
        MOTOR[i].rcurrent = PID[i].Pos.qOut;
    }
	
#ifdef DEVELOP_MODE
#ifdef LOG_POSLOOP
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        dataLOG1[dataLOGIdx] = PID[0].Pos.qdInRef;
        dataLOG2[dataLOGIdx] = MOTOR[0].mposition;
        //dataLOG3[dataLOGIdx] = (int16_t)PID[1].Pos.qdInRef;
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
    int i;
    for (i=0;i<3;i++) {
        NLFilter2Fx(&NLF[i].Out, &NLF[i].Status, NLF_vel_max, NLF_acc_max_shift, POS_LOOP_FcSHIFT);
        PID[i].Pos.qdInMeas = MOTOR[i].mposition;
        PID[i].Pos.qdInRef  = NLF[i].Out.qdX;
        CalcPID(&PID[i].Pos, &PID[i].flag.Pos);
        MOTOR[i].rcurrent = PID[i].Pos.qOut;
    }

#ifdef DEVELOP_MODE
#ifdef LOG_TRACKLOOP
// LOGS DATA FOR Data Monitor Control Interface (DMCI) of MPLAB IDE
    dataLOGdecim++;
    if(dataLOGdecim == LOGDECIM)
    { 
        dataLOG1[dataLOGIdx] = NLF[0].Out.qdX;
        dataLOG2[dataLOGIdx] = NLF[0].Out.qdXdot;
        //dataLOG3[dataLOGIdx] = (int16_t)PID[2].Pos.qdInRef;
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
        MOTOR[0].mvelocity -= (MOTOR[0].mcurrent_filt - MOTOR[0].mcurrent_offset) >> 4;
    else
        MOTOR[0].mvelocity += (MOTOR[0].mcurrent_filt - MOTOR[0].mcurrent_offset) >> 4;
    if(MOTOR[0].mvelocity > 550)
        MOTOR[0].mvelocity = 550;
    else if(MOTOR[0].mvelocity < -550)
        MOTOR[0].mvelocity = -550;
	if (((MOTOR[0].mposition / (encoder_ticks / 3600.0))<-200)&&(home_f.state!=0))
		MOTOR[0].mvelocity = 0;
    MOTOR[0].mposition += (int32_t)MOTOR[0].mvelocity;
#endif

#ifdef PROTO_BOARD
    MOTOR[0].mvelocity = PosCount[0];
    PosCount[0] = POS1CNT;// da modificare
    MOTOR[0].mvelocity -= PosCount[0];

	IC1PulseTmp = IC1Pulse;
	IC1PeriodTmp = IC1Period;
	IC1Pulse = 0;
	IC1Period = 0;
    
	if (IC1PulseTmp != 0)
	{
		MOTOR[0].velocityRPM = (int16_t)(((int32_t)(kvel*IC1PulseTmp))/IC1PeriodTmp);
		//MOTOR[0].velocityRPM_temp = (int16_t)(((int32_t)(kvel*IC1Pulse))/((IC1currentPeriod_temp+IC1previousPeriod_temp)/2));
	}
	else
		MOTOR[0].velocityRPM = 0;

    MOTOR[0].mposition += (int32_t)MOTOR[0].mvelocity;
#endif

//mtheta1 = MOTOR[1].mposition / (encoder_ticks / 3600.0);
//mtheta1 = RSH((MOTOR[1].mposition * decdeg_to_ticks_int),10); // deg to ticks is 5.10 fixed-point so RightSHift!
}

void UpdateEncoder2(void)
{
#ifdef SIMULATE
	if (home_f.state!=2)
	{
    if(DIR2)    
        MOTOR[1].mvelocity -= (MOTOR[1].mcurrent_filt - MOTOR[1].mcurrent_offset) >> 4;
    else
        MOTOR[1].mvelocity += (MOTOR[1].mcurrent_filt - MOTOR[1].mcurrent_offset) >> 4;
    if(MOTOR[1].mvelocity > 550)
        MOTOR[1].mvelocity = 550;
    else if(MOTOR[1].mvelocity < -550)
        MOTOR[1].mvelocity = -550;
	if (((MOTOR[1].mposition / (encoder_ticks / 3600.0))<-200)&&(home_f.state!=0))
		MOTOR[1].mvelocity = 0;
    MOTOR[1].mposition += (int32_t)MOTOR[1].mvelocity;
	}
#endif

#ifdef PROTO_BOARD
    MOTOR[1].mvelocity = PosCount2;
    PosCount2 = POS2CNT;
    MOTOR[1].mvelocity -= PosCount2;
	
	IC2PulseTmp = IC2Pulse;
	IC2PeriodTmp = IC2Period;
	IC2Pulse = 0;
	IC2Period = 0;

	if(IC2PulseTmp != 0)
	{
		MOTOR[1].velocityRPM = (int16_t)(((int32_t)(kvel*IC2PulseTmp))/IC2PeriodTmp);
//		MOTOR[1].velocityRPM_temp = (int16_t)(((int32_t)(kvel*IC2PulseTmp))/((IC2currentPeriod_temp+IC2previousPeriod_temp)/2));
	}
	else
		MOTOR[1].velocityRPM = 0;

    MOTOR[1].mposition += (int32_t)MOTOR[1].mvelocity;
#endif

//mtheta2 = MOTOR[1].mposition / (encoder_ticks / 3600.0);
//mtheta2 = RSH((MOTOR[1].mposition * decdeg_to_ticks_int),10); // deg to ticks is 5.10 fixed-point so RightSHift!
}

void UpdateEncoder3(void)
{
#ifdef SIMULATE
	if ((home_f.state!=2)&&(home_f.state!=3))
{
    if(DIR3)    
        MOTOR[2].mvelocity -= (MOTOR[2].mcurrent_filt - MOTOR[2].mcurrent_offset) >> 4;
    else
        MOTOR[2].mvelocity += (MOTOR[2].mcurrent_filt - MOTOR[2].mcurrent_offset) >> 4;
    if(MOTOR[2].mvelocity > 550)
        MOTOR[2].mvelocity = 550;
    else if(MOTOR[2].mvelocity < -550)
        MOTOR[2].mvelocity = -550;
	if (((MOTOR[2].mposition / (encoder_ticks / 3600.0))<-200)&&(home_f.state!=0))
		MOTOR[2].mvelocity = 0;
    MOTOR[2].mposition += (int32_t)MOTOR[2].mvelocity;
}
#endif

#ifdef PROTO_BOARD
    MOTOR[2].mvelocity = DownCount;
    MOTOR[2].mvelocity -=UpCount;
    if(MOTOR[2].direction_flags.encoder_chB_lead)
    {
        DownCount=TMR4;
        UpCount=TMR1;
    }
    else
    {
        DownCount=TMR1;
        UpCount=TMR4;
    }
    MOTOR[2].mvelocity+=UpCount;    
    MOTOR[2].mvelocity-=DownCount;
      
    MOTOR[2].mposition+=(int32_t)MOTOR[2].mvelocity; 
#endif

//mtheta3 = MOTOR[2].mposition / (encoder_ticks / 3600.0);
//mtheta3 = RSH((MOTOR[2].mposition * decdeg_to_ticks_int),10); // deg to ticks is 5.10 fixed-point so RightSHift!
}

void homing_manager(void)
{
    int i;
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
                    for(i=0;i<3;i++){
                        if(TRAJ[i].flag.enable && !TRAJ[i].flag.active)
				TRAJ[i].param.qdPosition = MOTOR[i].mposition;
			PosTRAJ(&TRAJ[i].param, &TRAJ[i].flag);
                    }
                    if((!TRAJ[0].flag.exec)&&(!TRAJ[1].flag.exec)&&(!TRAJ[2].flag.exec))
                    {
                        for(i=0;i<3;i++) {
                            //home.position[i] = MOTOR[i].mposition;
				TRAJ[i].param.qdPosition = MOTOR[i].mposition;
				TRAJ[i].param.qVelCOM = -home.Velocity;
                        }
                        home_f.state = 2;
                    }
                    break;
			
		case 2: //homing motor1
		case 3:	//homing motor2
		case 4: //homing motor3
                        i=home_f.state-2;
                        JogTRAJ(&TRAJ[i].param, &TRAJ[i].flag); //JOG motion
			//home.position[i] = TRAJ[i].param.qdPosition;
			if ((MyAbs(TRAJ[i].param.qdPosition - MOTOR[i].mposition)) > 1000)
			{
                            home_f.state++;
                            MOTOR[i].mposition = home.position[i];
                            TRAJ[i].param.qdPosition = home.position[i] + 100;
			}
			break;
		case 5:
                    home_f.done = 1;
                    home_f.homing_active = 0;
                    //theta1 = RSH((home.position[0] * decdeg_to_ticks_int),8); //-200;
                    //theta2 = RSH((home.position[1] * decdeg_to_ticks_int),8);//-200;
                    //theta3 = RSH((home.position[2] * decdeg_to_ticks_int),8);//-200;
                    //delta_calcForward(&angleJoints_temp);
			break;

			default: break;
	}
}

void update_delta_joints(void)
{
	angleJoints_actual.theta1 = convert_deg_to_rad(MOTOR[0].mposition * ticks_to_deg);
	angleJoints_actual.theta2 = convert_deg_to_rad(MOTOR[1].mposition * ticks_to_deg);
	angleJoints_actual.theta3 = convert_deg_to_rad(MOTOR[2].mposition * ticks_to_deg);
}

/*void update_delta_EE(void) //@XXX rimbalzo ad un altra funzione?? tantovale farlo prima
{
	delta_calcForward(&angleJoints_actual, &coordinates_actual);
}*/

