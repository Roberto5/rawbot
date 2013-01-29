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
 *    Filename:       main.c                                          *
 *    Date:           20/04/2011                                      *
 *    File Version:   0.2                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 
 *    Code Description
 *  
 *  This file contains the main() function.
 *
 ********************************************************************/
 
#include "sys_hw.h"
#include "extern_globals.h"
#include "generic_defs.h"
#include "Comms.h"
#include "PWM.h"
#include "ADC_DMA.h"
#include "QEI.h"
#include "Timers.h"
#include "PPS.h"
#include "SACT_Protocol.h"
#include "Controls.h"
#include "Trajectories.h"
#include "PID.h"
#include "geometry.h"
#include "motion.h"
#include "DEE Emulation 16-bit.h"
#include "InputCapture.h"
#include "Kinematix.h"

// CONFIGURATION BITS fuses (see dspic specific .h)
// i dispositivi son controllati dall'oscillatore interno, quindi scelgo FRC con moltiplicatore PLL
_FOSCSEL(FNOSC_FRCPLL);

//seleziono di non avere un clock dall'esterno e pongo i pin dell'oscillatore come pin di I/O
_FOSC(OSCIOFNC_ON & POSCMD_NONE);  
 
// non utilizzo il timer Watch-Dog.
_FWDT(FWDTEN_OFF & WINDIS_OFF);   
              
//spengo il timer di reset e configuro le polarità dei PWM
/**********************************************************
 * IMPORTANT: Low-side polarity should be inverted in order
 * to be compatible with the ADC synch strategy described
 * in PWM.c!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * On dsPIC33F the WriteConfig() function defined there 
 * DOES NOT WORK, so it is necessary to set polarity HERE
 * NOTE: don't care about high-side polarity, PWMxH pins
 * used as normal I/O
 **********************************************************/
_FPOR(PWMPIN_OFF & HPOL_OFF & LPOL_OFF & FPWRT_PWR1);

//imposto i pin di programmazione del DsPic in PGC3 PGD3
_FICD(JTAGEN_OFF & ICS_PGD3);

//Disabilito i codici di Protezione  e di protezione in scrittura
_FGS(GCP_OFF & GWRP_OFF);

// Function prototypes for "soft" real-time event handlers
void medium_event_handler(void);
void slow_event_handler(void);
void debounce_switches(void);
void control_mode_manager(void);
void diagnostics(void);

// Function prototype for parameters udpate
void update_params(void);

// WELCOME MESSAGE!
const unsigned char WelcomeMsg[] = 
{"\r\n---   DeltaBot ACTuator Board  ---\r\n"
     "-----   HW REV.1 - FW v1.4   -----\r\n"
     "Type the following sequence\r\n"
     "SYNC0+cr/lf SYNC1+cr/lf SYNCA+cr/lf\r\n"
     "to enter ASCII mode:\r\n"
};

/*******************************************************
* MAIN function, just setup some inits and loops
* "soft" real-time event handlers, defined hereafter
********************************************************/
int main(void)
{    
// configuro l'oscillatore interno che mi fornisce Tcy
// Fosc = Fin (M/(N1*N2))
// FCY = Fosc/2
    PLLFBD = 39; 			// M = 40
    CLKDIVbits.PLLPOST=0; 	// N2 = 2
    CLKDIVbits.PLLPRE=0; 	// N1 = 2

    RCONbits.SWDTEN = 0;	//disabilito il watchdog 

	DataEEInit();

    //Init Peripheral Pin Selection (QEI and UART)
    PPS_Init();
   
    control_flags.first_scan = 1;
    slow_ticks_limit = SLOW_RATE * (FCY_PWM / 1000) - 1 ;
    medium_ticks_limit = MEDIUM_RATE * (FCY_PWM / 1000) - 1;
    
    MOTOR[0].mposition = zero_pos1;//parto dalla posizione iniziale 90 90 90
    MOTOR[1].mposition = zero_pos2;
    MOTOR[2].mposition = zero_pos3;

	/*mtheta1 = 0;
	mtheta2 = 0;
	mtheta3 = 0;

	x_cart = 0;
	y_cart = 0;
	z_cart = 0;*/

    coordinates_actual.x = 0;
    coordinates_actual.y = 0;
    coordinates_actual.z = 0;

    coordinates_temp.x = 0;
    coordinates_temp.y = 0;
    coordinates_temp.z = 0;

    angleJoints_actual.theta1 = 0;
    angleJoints_actual.theta2 = 0;
    angleJoints_actual.theta3 = 0;

    angleJoints_temp.theta1 = 0;
    angleJoints_temp.theta2 = 0;
    angleJoints_temp.theta3 = 0;

    update_params();
    
    direction_flags_prev = direction_flags_word;
     // UARTs init
     // no need to set TRISx, they are "Module controlled"
    UART1_Init();  

    // Setup control pins and PWM module,
    // which is needed also to schedule "soft"
    // real-time tasks w/PWM interrupt tick counts
    
#ifdef BRIDGE_LAP
    PWM1=PWM2=PWM3=0;
    PWM1_TRIS = OUTPUT;
    PWM2_TRIS = OUTPUT;
    PWM3_TRIS = OUTPUT;
#else
    DIR1 = MOTOR[0].direction_flags.motor_dir;//0;
    DIR2 = MOTOR[1].direction_flags.motor_dir;//1;
    DIR3 = MOTOR[2].direction_flags.motor_dir;
    DIR1_TRIS = OUTPUT;
    DIR2_TRIS = OUTPUT;
    DIR3_TRIS = OUTPUT;
#endif

    
    
    //BRAKE1 = 0;
    //BRAKE2 = 0; 

    
    //BRAKE1_TRIS = OUTPUT;
    //BRAKE2_TRIS = OUTPUT;
    
    CURRSENSE1_TRIS = INPUT;
    CURRSENSE2_TRIS = INPUT;
    CURRSENSE3_TRIS = INPUT;
    
    PWM_Init();
    
    // MUST SETUP ALSO ANALOG PINS AS INPUTS
    AN0_TRIS = INPUT;
    AN1_TRIS = INPUT;
    AN2_TRIS = INPUT;
    
    ADC_Init();
    DMA0_Init();
    
    // SETUP ENCODER INPUTS
    // QEI inputs are "module controlled"
    // -> no need to set TRISx
    QEI1_Init();
    QEI2_Init();
    
    // Timers used to acquire Encoder 3
    // corresponding PINS set as inputs
    T1CK_TRIS = INPUT;
    T4CK_TRIS = INPUT;
    Timer1_Init();
    Timer2_Init();
    Timer4_Init();
	
    // Timer5 used to schedule POSITION loops
    Timer5_Init();

	//Input capture
    IC1_Init();
    IC2_Init();

    // TEST PIN
    TEST_PIN_TRIS = OUTPUT;
    TEST_PIN = FALSE;

    while(1)//a ciclo infinito ripeto queste 2 routine
        {	            
			medium_event_handler();
            slow_event_handler();
        }
    
    return 0; //code should never get here
}// END MAIN()

/*******************************************************
* Update parameters function, called at init and when
* a parameter is modified through SACT protocol
********************************************************/
void update_params(void)
{
    uint32_t templong;//intero lungo temporaneo
    uint8_t tempshift,i;//scalamento temporaneo

///////////////////////////////////////////////////////////////////
// VARIABLES FOR RUN-TIME USE OF PARAMETERS 
    max_current = parameters_RAM[0];//la corrente massima è in memoria RAM
    //encoder_counts_rev = (int32_t)parameters_RAM[19] << 2; // TAKE INTO ACCOUNT x4 QEI MODE
    //direction_flags.word = parameters_RAM[21];//anche la direzione è in memoria RAM

//ROBOT DIMENSION [in meters] 
    lf = parameters_RAM[13]/1000.0; //control arm lenght
    le = parameters_RAM[14]/1000.0; //forearm lenght
    e = parameters_RAM[15]/1000.0; //effector apothema
    f = parameters_RAM[16]/1000.0; //base apothema

//ROBOT LIMITS [in degrees]
    sphJLim = parameters_RAM[17]; //spheric joint limit
    posJLim = parameters_RAM[18]; //positive joint limit
    negJLim = parameters_RAM[19]; //negative joint limit

//encoder parameters
    encoder_ticks = (int32_t) parameters_RAM[25] * parameters_RAM[26] * 4;//10200

    kvel = (int32_t) ((float) 1/parameters_RAM[25] * FCY * 60); //4422000

    decdeg_to_ticks = encoder_ticks / 3600.0;

    decdeg_to_ticks_int = (uint16_t)((3600L << 10)/encoder_counts_rev); // in 5.10 fixed point
///////////////////////////////////////////////////////////////////
// CONTROL LOOPS and TRAJ PLANNERS INIT
////INIT PID CURRENT 1
    for (i=0;i<3;i++) {
    /*/ INIT PID Current
        PID[i].Current.qKp = parameters_RAM[5];
        PID[i].Current.qKi = parameters_RAM[6];
        PID[i].Current.qKd = parameters_RAM[7];
        PID[i].Current.qN  = parameters_RAM[8]; // SHIFT FINAL RESULT >> qN
        PID[i].Current.qdOutMax =  (int32_t)(FULL_DUTY << (PID[i].Current.qN-1));
        PID[i].Current.qdOutMin = -(int32_t)(FULL_DUTY << (PID[i].Current.qN-1));

        InitPID(&PID[i].Current, &PID[i].flag.Current,-1);*/
    // INIT PID Position
        PID[i].Pos.qKp = parameters_RAM[9];
        PID[i].Pos.qKi = parameters_RAM[10];
        PID[i].Pos.qKd = parameters_RAM[11];
        PID[i].Pos.qN  = parameters_RAM[12];  // SHIFT FINAL RESULT >> qN
        //PID[i].Pos.qdOutMax = ((int32_t)max_current << PID[i].Pos.qN);
        //PID[i].Pos.qdOutMin = -PID[i].Pos.qdOutMax;
        PID[i].Pos.qdOutMax =  (int32_t)(FULL_DUTY << (PID[i].Pos.qN-1));
        PID[i].Pos.qdOutMin = -(int32_t)(FULL_DUTY << (PID[i].Pos.qN-1));


        InitPID(&PID[i].Pos, &PID[i].flag.Pos,0);
    ////INIT TRAJ PLANNER 1
        TRAJ[i].flag.enable = 0;
        TRAJ[i].param.qVLIM = parameters_RAM[1];
        TRAJ[i].param.qACC = parameters_RAM[2];
        TRAJ[i].param.qVELshift = parameters_RAM[3];
        TRAJ[i].param.qACCshift = parameters_RAM[4];
        TRAJ[i].param.qdPosition = parameters_RAM[25];
    }

////INIT Limits for nonlinear filter
    // vel max = (VLIM for PosTRAJ >> VEL SCALE Shift) * POS_LOOP_FREQ
    templong = parameters_RAM[1] >> parameters_RAM[3];
    NLF_vel_max = templong << POS_LOOP_FcSHIFT;
    // acc max = ((ACC for PosTRAJ >> ACC SCALE Shift) * POS_LOOP_FREQ^2 ) >> VEL SCALE Shift
    templong = parameters_RAM[2] >> parameters_RAM[4]; 
    templong = templong << (POS_LOOP_FcSHIFT<<1) ;
    templong = templong >> parameters_RAM[3];
    // search best 2^N scaling approximation for (acc max)^-1
    tempshift = 0;
    while((1UL<<tempshift) <= templong)
        tempshift++;
    NLF_acc_max_shift = tempshift-2;
   
//INIT HOMING!!!!!!!!!!!!!!!!!!!!!!!!!
    home_f.done = 0;
home.Velocity = parameters_RAM[1] / parameters_RAM[20];
home.position[0] = -(int32_t)parameters_RAM[22];
home.position[1] = -(int32_t)parameters_RAM[23];
home.position[2] = -(int32_t)parameters_RAM[24];
home.offsetPosition = parameters_RAM[21]; 
//-------------------------------------
}

/*******************************************************
* "Soft" real-time event handler for medium rate
********************************************************/
void medium_event_handler(void)
{
	    if(medium_event_count > medium_ticks_limit)
    {
        medium_event_count = 0;
        
        diagnostics(); 

////////UARTs Receive management for SACT protocol
        U1_SACT_Parser();
        U2_SACT_Parser();
    }// END IF medium_event_count..
}// END medium_event_handler

/****************************************************
* This function manages diagnostic checks: 
* overcurrents, tracking errors, etc.
****************************************************/
void diagnostics(void)
{
    static uint8_t overcurrent_count[3]={0,0,0},i;

    for (i=0;i<3;i++) {
        // ACCUMULATE (SORT OF I^2T)
        if(MOTOR[i].mcurrent_filt > (max_current + 100))
            overcurrent_count[i]++;
        else
            overcurrent_count[i] = 0;
        if(overcurrent_count[i] > 5)
        {
        // OVERCURRENT:
            status_flags.overcurrent[i] = 1;
        }
    }
    
    
    // Mask off ALL flags BUT motor faults and set board to OFF_MODE
    if(((status_flags.dword & 0x000000FF) != 0)&&(control_mode.state != OFF_MODE))
    {
// switch off immediately and raise OFF_MODE req.
//       control_flags.current_loop_active = 0;
//       control_flags.pos_loop_active = 0;
//       P1DC1 = FULL_DUTY;
//       P1DC2 = FULL_DUTY;
        
        control_mode.off_mode_req = 1;
    }
} // END diagnostics

/*******************************************************
* "Soft" real-time event handler for slow rate
********************************************************/
void slow_event_handler(void)
{
    if(slow_event_count > slow_ticks_limit)
    {
        slow_event_count = 0;
        
        if(control_flags.first_scan)
        {
            putsUART((unsigned char *)WelcomeMsg,&UART1);
            //putsUART((unsigned char *)WelcomeMsg,&UART2);
            
            control_flags.first_scan = 0;
        }

        // (RAM) Parameters update management
        if(control_flags.PAR_update_req)
        {
            update_params();
            control_flags.PAR_update_req = 0;
        }
        
        if(direction_flags_word != direction_flags_prev)
        {
            // RESET COUNTS
            QEI1_Init();
            QEI2_Init();
            Timer1_Init();
            Timer4_Init();
            direction_flags_prev = direction_flags_word;
        }

        // EEPROM update management
        if(control_flags.EE_update_req)
        {
			control_flags.EE_update_req = 0;
        }

        update_delta_joints();
        delta_calcForward(&angleJoints_actual, &coordinates_actual);
		//update_delta_EE();//aggiornamento delle strutture dati
        status_flags.homing_done = home_f.done;

        // SACT protocol timeout manager (see SACT_protocol.c)
        SACT_timeout();
		SACT_SendSDP();
		SACT_SendSSP();

        // CONTROL MODE STATE MANAGER
        control_mode_manager();

    } // END IF slow_event_count..
}// END slow_event_handler

/****************************************************
* This function manages the control mode state and
* transitions, including necessary PID init, resets..
****************************************************/
void control_mode_manager(void)
{ 
    int i;
    switch(control_mode.state)
    {
//////////////////////////////////////////////////////////////////////
//  OFF MODE
        case OFF_MODE :
            
            for(i=0;i<3;i++) {
                TRAJ[i].flag.enable = 0;
                TRAJ[i].flag.active = 0;
                TRAJ[i].flag.exec = 0;
                TRAJ[i].flag.busy = 0;
                MOTOR[i].rcurrent=0;
                MOTOR[i].rcurrent_req=0;
            }
            control_flags.current_loop_active = 0;
            control_flags.pos_loop_active = 0;
            home_f.state = 0;
            //	home_f.done = 0;
            P1DC1 = FULL_DUTY/2;
            P1DC2 = FULL_DUTY/2;
            P2DC1 = FULL_DUTY/2;
            PWM1=PWM2=PWM3=FALSE;
            //	x_cart = 0.0;
            //	y_cart = 0.0;
            //	z_cart = 0.0;
            // STATE TRANSITIONS
                            if(control_mode.ax_pos_mode_req)
                            {   
                                control_mode.state = AX_POS_MODE;
                            }
                            // STATE TRANSITIONS
                            else if(control_mode.torque_mode_req)
                                {
                                    control_mode.state = TORQUE_MODE;    
                                }
                            else if(control_mode.cart_mode_req)
                                {
                                    control_mode.state = CART_MODE;
                                }
                            else if(control_mode.track_mode_req)
                                {
                                    control_mode.state = TRACK_MODE;
                                    for (i=0;i<3;i++) {
                                        InitNLFilter2Fx(&NLF[i].Out, &NLF[i].Status);
                                        NLF[i].Status.qdRcommand = MOTOR[i].mposition;
                                        NLF[i].Status.qdRprev = MOTOR[i].mposition;
                                        NLF[i].Status.qdXint = MOTOR[i].mposition;
                                        NLF[i].Status.MODE = 1;
                                    }
                                }
                            // IF there is ANY transition, RESETS PIDs
                            if(control_mode.trxs)
                            {
                                PWM1=PWM2=PWM3=TRUE;
                                // RESET MOTOR FAULT FLAGS (first byte)
                                status_flags.dword = status_flags.dword & 0xFFFFFF00;
                                
                                //RESETS PIDs
                                for(i=0;i<3;i++) {
                                    InitPID(&PID[i].Current, &PID[i].flag.Current,-1);
                                    InitPID(&PID[i].Pos, &PID[i].flag.Pos,0);
                                }
                                control_mode.trxs = 0;
                            }
                            
                            break;
/////////////////////////////////////////////////////////////////////
//  TORQUE MODE 1
        case TORQUE_MODE : control_flags.current_loop_active = 1;

                        // STATE TRANSITIONS
                            if(control_mode.off_mode_req)
                            {
                                control_mode.state = OFF_MODE;
                                control_mode.trxs = 0;
                            }
                            break;
/////////////////////////////////////////////////////////////////////
//  AXIS POSITION MODE
        case AX_POS_MODE : control_flags.current_loop_active = 1;
                           control_flags.pos_loop_active = 1;
						//abilito i 3 motori
                           TRAJ[0].flag.enable = 1;
                           TRAJ[1].flag.enable = 1;
                           TRAJ[2].flag.enable = 1;
                           
                        // STATE TRANSITIONS
                           if(control_mode.off_mode_req)
                            {
                                control_mode.state = OFF_MODE;
                                control_mode.trxs = 0;
                            }
                                
                            break;
/////////////////////////////////////////////////////////////////////
//  CART MODE
        case CART_MODE : control_flags.current_loop_active = 1;
                         control_flags.pos_loop_active = 1;

                         TRAJ[0].flag.enable = 1;
                         TRAJ[1].flag.enable = 1;
                         TRAJ[2].flag.enable = 1;

                         // STATE TRANSITIONS
                         if(control_mode.off_mode_req)
                         {
                            control_mode.state = OFF_MODE;
                            control_mode.trxs = 0;
                         }
                         break;
/////////////////////////////////////////////////////////////////////
//  TRACK MODE
        case TRACK_MODE : control_flags.current_loop_active = 1;
                         control_flags.track_loop_active = 1;

                         // STATE TRANSITIONS
                         if(control_mode.off_mode_req)
                         {
                            control_mode.state = OFF_MODE;
                            control_mode.trxs = 0;
                         }
                         break;
/////////////////////////////////////////////////////////////////////
//  ERROR!!!!
        default    :    break;
    }//end SWITCH   
    
}// END control mode manager
