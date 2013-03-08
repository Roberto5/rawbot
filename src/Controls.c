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

pid PID[N_MOTOR];
// TRAJ parameters and flags structures
traj TRAJ[N_MOTOR];
// nonlinear filter smoothing

nlf NLF[N_MOTOR];
//Homing
tHome home;
tHomeflags home_f;

// limits
uint32_t NLF_vel_max;
uint32_t NLF_acc_max_shift;

//temporary variable for input capture
volatile int32_t ICPeriodTmp[3];
volatile int16_t ICPulseTmp[3];

/************************************************
 * LOCAL FUNCTIONS
 ***********************************************/

void UpdateEncoder(void);

void homing_manager(void);
/***********************************************
 * LOCAL VARIABLES
 ***********************************************/
// Temps for odometry calc.
int16_t UpCount,DownCount,PosCount[2];


// TO MANAGE LMD18200 SIGN INVERSION
#define BLANKS 2
uint8_t DIR1_PREV, DIR2_PREV,DIR3_PREV;
uint8_t DIR_TMP[N_MOTOR];
uint8_t tempidx;

/*************************************
 * Current control loops
 * for all motors
 *************************************/
void CurrentLoops(void)
{
    int i,duty[3];
    for (i=0;i<N_MOTOR;i++) {
#ifdef BRIDGE_LAP
        // MANAGE SIGN OF MEASURE (locked anti-phase control of LMD18200)
#ifdef RAW_POWER
        PID[i].Current.qdInRef = (int32_t)(MOTOR[i].rcurrent);
#else
        if (MOTOR[i].rcurrent<0)
            PID[i].Current.qdInRef=-(int32_t) MOTOR[i].rcurrent;
        else
            PID[i].Current.qdInRef=(int32_t)MOTOR[i].rcurrent;
#endif    
        PID[i].Current.qdInMeas=(int32_t)(MOTOR[i].mcurrent_filt);

    CalcPI(&PID[i].Current, &PID[i].flag.Current);
#ifdef RAW_POWER
#else
    PID[i].Current.qOut+=ZERO_DUTY/2;
#endif
    if(MOTOR[i].direction_flags.motor_dir 
#ifdef RAW_POWER
#else
            ^ (MOTOR[i].rcurrent<0)
#endif
            )
        duty[i] = ZERO_DUTY - PID[i].Current.qOut; // INVERTED FIRING!
    else
        duty[i] = ZERO_DUTY + PID[i].Current.qOut;
    //duty[i]=ZERO_DUTY;
#else
    // FIRST MOTOR
    // "Standard" bipolar PID control, no offset, since ACS714 is used
    PID[i].Current.qdInRef  = (int32_t)MOTOR[i].rcurrent;
    //if(!MOTOR[i].direction_flags.motor_dir)
        PID[i].Current.qdInMeas = (int32_t)(MOTOR[i].mcurrent_filt);
    //else
      //  PID[i].Current.qdInMeas = -(int32_t)(MOTOR[i].mcurrent);
    //PIDCurrent1.qdInMeas = (int32_t)(mcurrent1 - mcurrent1_offset);
    CalcPI(&PID[i].Current, &PID[i].flag.Current);
    if(PID[i].Current.qOut < 0) {
        DIR_TMP[i] = ~MOTOR[i].direction_flags.motor_dir;
    }
    else {
        DIR_TMP[i] = MOTOR[i].direction_flags.motor_dir;
    }
    duty[i]=FULL_DUTY- (int16_t)(PID[i].Current.qOut<0 ? -PID[i].Current.qOut : PID[i].Current.qOut);

#endif
    }
    // IMPORTANT: INVERTED FIRING!!
    
#ifdef RAW_POWER
   P2DC1=duty[0];
#ifdef BRIDGE_LAP
#else
   DIR1 = DIR_TMP[0];
#endif
#else
   P1DC1=duty[0];
   P1DC2=duty[1];
   P2DC1=duty[2];
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
        for (i=0;i<N_MOTOR;i++) {
            if(TRAJ[i].flag.enable && !TRAJ[i].flag.active) //if enabled but not active
                TRAJ[i].param.qdPosition = MOTOR[i].mposition;
            PosTRAJ(&TRAJ[i].param, &TRAJ[i].flag);//trapezoidal motion
        }
    }
    for (i=0;i<N_MOTOR;i++) {
        PID[i].Pos.qdInMeas = MOTOR[i].mposition;
        PID[i].Pos.qdInRef  = TRAJ[i].param.qdPosition;
        CalcPID(&PID[i].Pos, &PID[i].flag.Pos);
        MOTOR[i].rcurrent = PID[i].Pos.qOut;
    }
#ifdef BY_PASS_CURRENT_LOOP
    if(PID[0].Pos.qOut < 0) {
            DIR1 = ~MOTOR[0].direction_flags.motor_dir;
    }
    else {
        DIR1 = MOTOR[0].direction_flags.motor_dir;
    }
    P2DC1=FULL_DUTY- (int16_t)(PID[0].Pos.qOut<0 ? -PID[0].Pos.qOut : PID[0].Pos.qOut);
#endif 
	
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
    for (i=0;i<N_MOTOR;i++) {
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
 * Function to update encoder counts
 * for ALL motors
 *************************************/


void UpdateEncoder(void)
{
    int i,poscnt[2],tot;
#ifdef SIMULATE
    for (i=0;i<N_MOTOR;i++) {
            MOTOR[i].mvelocity += (MOTOR[i].mcurrent) >> 4;
        if(MOTOR[i].mvelocity > 550)
            MOTOR[i].mvelocity = 550;
        else if(MOTOR[i].mvelocity < -550) MOTOR[i].mvelocity = -550;
	if (((MOTOR[i].mposition / (encoder_ticks / 3600.0))<-200)&&(home_f.state!=0))
            MOTOR[i].mvelocity = 0;
        MOTOR[i].mposition += (int32_t)MOTOR[i].mvelocity;
    }
    
#endif
    poscnt[0]=POS1CNT;
    poscnt[1]=POS2CNT;
#ifdef PROTO_BOARD
    tot=N_MOTOR>N_QEI ? N_QEI : N_MOTOR ;
    for (i=0;i<tot;i++) {
        MOTOR[i].mvelocity = PosCount[i];
        PosCount[i] = poscnt[i];// da modificare
        MOTOR[i].mvelocity -= PosCount[i];
        ICPulseTmp[i] = IC_Pulse[i];
        ICPeriodTmp[i] = IC_Period[i];
        IC_Pulse[i] = 0;
        IC_Period[i] = 0;
        if (ICPulseTmp[i] != 0) {
            MOTOR[i].velocityRPM = (((int32_t)(kvel*ICPulseTmp[i]))/ICPeriodTmp[i]);
	//MOTOR[0].velocityRPM_temp = (int16_t)(((int32_t)(kvel*IC1Pulse))/((IC1currentPeriod_temp+IC1previousPeriod_temp)/2));
        }
        else
            MOTOR[i].velocityRPM = 0;
        MOTOR[i].mposition += (int32_t)MOTOR[i].mvelocity;
    }
    //motor 3 use a timer
#ifdef RAW_POWER
#else
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
    
#endif
}
void homing_manager(void)
{
    int i;
    int8_t b=1;
	switch (home_f.state)
	{
		case 0:
                    for (i=0;i<N_MOTOR;i++)
			angleJoints_temp[i] = 0;
                    move(angleJoints_temp);
                    home_f.state = 1;
		break;
		case 1:
                    for(i=0;i<N_MOTOR;i++){
                        if(TRAJ[i].flag.enable && !TRAJ[i].flag.active)
				TRAJ[i].param.qdPosition = MOTOR[i].mposition;
			PosTRAJ(&TRAJ[i].param, &TRAJ[i].flag);
                    }
                    for (i=0;i<N_MOTOR;i++)
                        b=b&& !TRAJ[i].flag.exec;
                    if(b)
                    {
                        for(i=0;i<N_MOTOR;i++) {
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
		break;
            default: break;
	}
}

void update_delta_joints(void)
{
    int i;
    for (i=0;i<N_MOTOR;i++)
	angleJoints_actual[i] = convert_deg_to_rad(MOTOR[i].mposition * ticks_to_deg);
}

