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
 *    Author: Marcello Bonfe'                                         *
 *                                                                    *
 *    Filename:       SACT_Protocol.c                                 *
 *    Date:           28/12/2010                                      *
 *    File Version:   0.1                                             *
 *    Compiler:       MPLAB C30 v3.23                                 *
 *                                                                    *
 **********************************************************************
 *    Code Description
 *  
 *  This file contains the management of SACT interface protocol
 *  and all the related configuration parameters.
 *
 **********************************************************************/

#include "sys_hw.h"
#include "generic_defs.h"
#include "extern_globals.h"
#include "Comms.h"
#include "SACT_Protocol.h"
#include "lib_crc.h"
#include "Kinematix.h"
#include "Trajectories.h" //for TRAJ parm/flags data-types
#include "QEI.h"
#include "geometry.h"
#include "demo.h"
#include "PPS.h"
#include "motion.h"
#include "DEE Emulation 16-bit.h"
#include "Convert.h"
#include <string.h> //for memcmp()
#include <stdlib.h> //for atol()
                    //NOTE: itoa() is not supported natively by C30!!!
#include <limits.h> //for INT_MAX and INT_MIN
#include <math.h>	//for math functions

/**********************************************
 * COMMANDS / PARAMETERS DEFINITION:
 *********************************************/
/*NUOVA STRUTTURA COMMANDI
	commando 5 sostituire l'azione dimostrazione con l'azione movimento ad incremento dalla posizione corrente
	commandi 6-7 non utilizzato perchè riferiti all'azione di una pinza
	commando 10 odometria serve solo nel caso di robot mobili
	commandi 21-22-23 non ho controllo PID della velocità
	commandi da 29 a 36 non utilizzati perchè legati anch'essi al robot mobile

	comandi da inserire:
	-esistono azioni non ancora elaborate?
	-struttura meccanica del robot (lunghezza giunto superiore, lunghezza giunto inferiore, punto centrale base, punto centrale base fissa, punto centrale base mobile)
*/
const t_command_data command_data [N_COMMANDS+N_PARAMS] =    {
// Min,Max,Args, Line1 msg, Quick msg 
//Azioni
{0,0,0,             "Disconnect SACT ","DIS"},//0
{0,0,1,             "CONTROL MODE    ","CMO"},//1
{0,0,N_MOTOR,             "SET TORQUE Refs.","STR"},//2
{0,0,N_MOTOR,             "SET AX.POS.Refs.","SPR"},//3
{0,0,3,             "SET CART POINTS ","SCP"},//4
{0,0,3,             "MOVE INCREMENT  ","MOI"},//5 
{0,0,0,             "HOME            ","HOM"},//6 inizializzazione parametri da passare 1--> noi siamo obbligati a fare una procedura di homing hard-stop ma se ne potrebbero implementare altre
{0,0,1,             "GRIP            ","GRP"},//7 grip
{0,0,0,             "PULSE (BIN.only)","PUL"},//8
{0,0,0,             "UPDATE EEPROM   ","UEE"},//9
{0,0,1,             "SET SSP FLAGS   ","SSF"},//10
//parametri di funzionamento
{1,32767,1,         "MAX CURRENT     ","MXC"},//11
{1,32767,1,         "MAX VELOCITY    ","MXV"},//12
{1,32767,1,         "MAX ACCELERATION","MXA"},//13
{0,15,1,            "VEL. SCALING N. ","VSN"},//14
{0,15,1,            "ACC. SCALING N. ","ASN"},//15
{0,32767,1,         "CURR.Loop P GAIN","CLP"},//16
{0,32767,1,         "CURR.Loop I GAIN","CLI"},//17
{0,32767,1,         "CURR.Loop D GAIN","CLD"},//18
{0,15,1,            "CURR.Loop SCALE ","CLS"},//19
{0,32767,1,         "POS. Loop P GAIN","PLP"},//20
{0,32767,1,         "POS. Loop I GAIN","PLI"},//21
{0,32767,1,         "POS. Loop D GAIN","PLD"},//22
{0,100,1,            "POS. Loop SCALE ","PLS"},//23
//parametri meccanici
{0,32767,1,			"CONTROL ARM LENG","CAL"},//24 LUNGHEZZA BRACCIO SUPERIORE
{0,32767,1,			"FOREARM LENGHT	 ","FAL"},//25 LUNGHEZZA BRACCIO INFERIORE
{0,32767,1,			"BASE APOTHEM	 ","BAP"},//26 APOTEMA BASE FISSA
{0,32767,1,			"EFFECTOR APOTHEM","EAP"},//27 APOTEMA BASE MOBILE
//parametri di controllo
{0,32767,1,			"SPH. JOINT LIMIT","SJL"},//28 LIMITE ANGOLO GIUNTO SFERICO
{0,32767,1,			"JOINT POS LIMIT ","JPL"},//29 LIMITE POSITIVO GIUNTO
{0,32767,1,			"JOINT NEG LIMIT ","JNL"},//30 LIMITE NEGATIVO GIUNTO
//parametri procedura di homing
{0,32767,1,			"VEL.PROP.FACTOR ","VPF"},//31 FATTORE PROPORZIONALE DI VELOCITA'
{0,32767,1,			"POSITION ERROR  ","POE"},//32 ERRORE TRA LA POSIZIONE MISURATA E LA STIMA DI POSIZIONE
//limiti meccanici
{0,32767,1,			"MOTOR 1 LIMIT   ","M1L"},//33 LIMITE MECCANICO DEL MOTORE 1
{0,32767,1,			"MOTOR 2 LIMIT   ","M2L"},//34 LIMITE MECCANICO DEL MOTORE 2
{0,32767,1,			"MOTOR 3 LIMIT   ","M3L"},//35 LIMITE MECCANICO DEL MOTORE 3
//parametri encoder
{0,32767,1,			"ENCODER STEP    ","EST"},//36 PASSI ENCODER MOTORE
{0,32767,1,			"GEAR RATIO	     ","GRA"},//37 RAPPORTO DI RIDUZIONE

{-32767,32767,1,         "mcurrent_offset 1","CO1"},//38
{-32767,32767,1,         "mcurrent_offset 2","CO2"},//39
{-32767,32767,1,         "mcurrent_offset 3","CO3"},//40
{0,1,1,         "motor_dir","MDR"},//41
};

// PARAMETERS stored in RAM.. default values..
uint16_t parameters_RAM[N_PARAMS]=
{    
    150,            // 0: MAX CURRENT (Command 11)
    10000,          // 1: MAX VELOCITY (Command 12)
    5000,          // 2: MAX ACCELERATION (Command 13)
    6,              // 3: VELOCITY SCALING SHIFT (Command 14)
    7,              // 4: ACCELERATION SCALING SHIFT (Command 15)
    200,            // 5: CURRENT LOOP P GAIN (Command 16)
    30,             // 6: CURRENT LOOP I GAIN (Command 17)
    0,              // 7: CURRENT LOOP D GAIN (Command 18)
    5,              // 8: CURRENT LOOP SCALING SHIFT (Command 19)
    100,            // 9: POSITION LOOP P GAIN (Command 20)
    1,             // 10: POSITION LOOP I GAIN (Command 21)
    0,              // 11: POSITION LOOP D GAIN (Command 22)
    12,             // 12: POSITION LOOP SCALING SHIFT (Command 23)
    180,            //13: CONTROL ARM LENGHT (Command 24) in millimeters
	398,			//14: FOREARM LENGHT (Command 25) in millimeters
	70,				//15: BASE APOTHEMA (Command 26) in millimeters
	43,				//16: EFFECTOR APOTHEMA (Command 27) in millimeters
	20,              // 17: SPHERICAL JOINT ANGLE LIMIT (Command 28) in degrees
	90,              // 18: POSITIVE JOINT LIMIT (Command 29) in degrees
  	20,              // 19: NEGATIVE JOINT LIMIT (Command 30) in degrees
    10,              // 20: VELOCITY PROPORTIONAL FACTOR (Command 31)//fattore proporzionale
    100,              // 21: POSITION ERROR (Command 32)//errore
    11700,              // 22: MECHANIC LIMIT MOTOR 1 (Command 33)
    11700,              // 23: MECHANIC LIMIT MOTOR 2 (Command 34)
    11700,              // 24: MECCANIC LIMIT MOTOR 3 (Command 35)
   	300,              // 25: ENCODER STEP (Command 36)
    30,              // 26: GEAR RATIO (Command 37)
    2027,              //27: mcurrent_offset 1 (Command 38)
    0,              //28: mcurrent_offset 2 (Command 39)
    0,              //29: mcurrent_offset 3 (Command 40)
    0               //30:MOTOR direction_flag (Command 41)
};

// HELP MESSAGES
const unsigned char ErrorSpace[] = {"\r\nPoint out of space!\r\n"};
const unsigned char ErrorJoints[] = {"\r\nBreaking joints!\r\n"};
const unsigned char ErrorMsg[] = {"\r\nIncorrect Command! Type '??' for command set.\r\n"};
const unsigned char ErrorParaMsg[] = {"\r\nIncorrect Command! Wrong N. of parameters.\r\n"};
const unsigned char ErrorParaLengthMsg[] = {"\r\nIncorrect Command! Wrong param. length.\r\n"};
const unsigned char ErrorParaLimitMsg[] = {"\r\nError: parameter out of range!\r\n"};
const unsigned char ErrorControlModeMsg[] = {"\r\nError: command invalid in current "};
const unsigned char CRLFMsg[] = {"\r\nCommands must be terminated with CR/LF sequence...\r\n"};
const unsigned char SyncMsg[] = {"\r\nWaiting for SYNC0+cr/lf SYNC1+cr/lf SYNCA+cr/lf\r\n"};
const unsigned char ModeErrorMsg[] = {"\r\nControl Mode active! No params update allowed.\r\n"};
const unsigned char CommandParaHeader[] = {"\r\nAction Commands:\r\n"};
const unsigned char MotorParaHeader[] = {"\r\nMotor Parameters:\r\n"};
const unsigned char RobotParaHeader[] = {"\r\nRobot Parameters:\r\n"};
const unsigned char ControlParaHeader[] = {"\r\nControl Parameters:\r\n"};
const unsigned char IOParaHeader[] = {"\r\nHW I/Os Parameters:\r\n"};
const unsigned char ParaHeader[] = 
            {"Description\t\tAbbreviation\t\tCurrent Value\r\n"};
const unsigned char ParaHeader_cmd[] = 
            {"Description\t\tAbbreviation\t\tArguments\r\n"};

const unsigned char HelpMsg_data [MAX_HELPMSG][32] =    
{    
    {"For Action Commands\tType '?A'\r\n"},
    {"For Motor Params\tType '?M'   \r\n"},
    {"For Robot Params\tType '?R'   \r\n"},
    {"For Control Params\tType '?C' \r\n"},
    {"For HW I/Os Params\tType '?I' \r\n"},
};

const unsigned char ControlModeMsg [6][15] =    
{    
    {"OFF_MODE    \r\n"},
    {"TORQUE_MODE\r\n"},
    {"AX_MODE	  \r\n"},
    {"CART_MODE   \r\n"},
};

const unsigned char FaultMsg[6][30] =
{    
    {"FAULT: overcurrent motor 1 \r\n"},
    {"FAULT: overcurrent motor 2 \r\n"},
	{"FAULT: overcurrent motor 3 \r\n"},
    {"FAULT: track error motor 1 \r\n"},
    {"FAULT: track error motor 2 \r\n"},
	{"FAULT: track error motor 3 \r\n"},
};

// TABLE ASSOCIATING EACH COMMAND/PARAM TO RELATED GROUP FOR HELP INFO:
// - each line corresponds to a group
// - each number in a line corresponds to the index of a command/parameter
//   associated to that group
const uint8_t help_info[MAX_HELPMSG][15] =
{
    {0,1,2,3,4,5,6,7,8,9,10,50,50,50,50}, // COMMANDS
    {11,12,13,14,15,38,39,40,41,50,50,50,50,50,50}, // MOTOR
    {24,25,26,27,28,29,30,50,50,50,50,50,50,50,50}, // ROBOT
    {16,17,18,19,20,21,22,23,31,32,36,37,50,50,50}, // CONTROL
    {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50}, // HW I/Os
};

// LOCAL VARIABLES

unsigned char rx1buf[MAX_ASCIILEN];
uint8_t rx1cnt = 0;
unsigned char rx2buf[MAX_ASCIILEN];
uint8_t rx2cnt = 0;
unsigned char BINRXbuf[MAX_ASCIILEN];
uint8_t BINRXcnt = 0;
unsigned char BINTXbuf[MAX_ASCIILEN];
uint8_t BINTXcnt = 0;
unsigned char u1temp = 0;
unsigned char u1prev = 0;
unsigned char u2temp = 0;
unsigned char u2prev = 0;

int8_t status;

//x_cart = 0.0 ,y_cart = 0.0 ,z_cart = 0.0;

uint8_t SACT_state = SACT_NOSYNC;
uint8_t SYNC_U1_step = 0;
uint8_t SYNC_U2_step = 0;

uint8_t BINLastCommand;

tSACT_flags SACT_flags;

tSSP_config SSP_config;

// LOCAL FUNCTIONS
void process_SYNC_U1(void);
void process_SYNC_U2(void);
// for ASCII mode
void process_ASCII(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg);
void CheckHelp(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg);
void SendHelpInfo(uint8_t table,volatile UART *ureg);
unsigned char GetMsgIndex(unsigned char *rxbuf);
void GetParamASCII(uint8_t idx, volatile UART *ureg);
void GetParamBIN(uint8_t idx, volatile UART *ureg);

// for ASCII mode
void process_BIN(unsigned char *rxbuf, uint8_t rxcnt);
void ParseBINCommand(void);


void ExecCommand(uint8_t idx,int16_t *args);


/****************************************
 * UART1 Buffer parser for SACT Protocol
 ***************************************/
void U1_SACT_Parser(void)
{

while(u1bufhead != u1buftail)
{
    u1prev = u1temp;
    u1temp = u1tmpbuf[u1buftail++];

    
    switch(SACT_state)
    {
//////////////////////////////////////////////////////////////////////
// NO SYNC STATE
        case SACT_NOSYNC:    rx1buf[rx1cnt++]=u1temp;
            switch(u1temp)
            {
                case CR : SACT_flags.cr_rec1 = 1;break;
                case LF : if (u1prev == CR) {
                        // TERMINATE STRING
                        rx1buf[rx1cnt]=NULLC;
                        process_SYNC_U1();
                        rx1cnt = 0;
                        SACT_flags.cr_rec1 = 0;
                    }
                    else {
                        putsUART((unsigned char*)CRLFMsg,&UART1);
                        rx1cnt = 0;
                    }
                break;
                default : //rx1buf[rx1cnt++]=u1temp;
                    if(rx1cnt > (MAX_ASCIILEN-1)) {
                        putsUART((unsigned char*)SyncMsg,&UART1);
                        rx1cnt = 0;
                    }
                    if(SACT_flags.cr_rec1) {
                        putsUART((unsigned char*)CRLFMsg,&UART1);
                        rx1cnt = 0;
                        SACT_flags.cr_rec1 = 0;
                    }
               break;
            }
        break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART1
        case SACT_ASCII_U1: rx1buf[rx1cnt++]=u1temp;
            switch(u1temp) {
                case CR : SACT_flags.cr_rec1 = 1;break;
                case LF : if (u1prev == CR) {
                        // TERMINATE STRING
                        rx1buf[rx1cnt]=NULLC;
                        process_ASCII(rx1buf,rx1cnt,&UART1);
                        rx1cnt=0;
                        SACT_flags.cr_rec1 = 0;
                    }
                    else {
                        putsUART((unsigned char*)CRLFMsg,&UART1);
                        rx1cnt = 0;
                    }
                break;
                default : //rx1buf[rx1cnt++]=u1temp;
                    if(rx1cnt > (MAX_ASCIILEN-1)) {
                        putsUART((unsigned char*)ErrorMsg,&UART1);
                        rx1cnt = 0;
                    }
                    if(SACT_flags.cr_rec1) {
                        putsUART((unsigned char*)CRLFMsg,&UART1);
                        rx1cnt = 0;
                        SACT_flags.cr_rec1 = 0;
                    }
               break;
            }
        break;
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART1
        case SACT_BIN_U1:
            rx1buf[rx1cnt++]=u1temp;
            process_BIN(rx1buf,rx1cnt);
            if(SACT_flags.valid_idx) {
                rx1cnt = 0; //Reset count if a command has been processed!
                SACT_flags.valid_idx = 0;
            }
        break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART2, NOT MANAGED HERE (see U2RXInterrupt) 
        case SACT_ASCII_U2:
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART2, NOT MANAGED HERE (see U2RXInterrupt)
        case SACT_BIN_U2:
            U1TXREG = u1temp;
            rx1cnt = 0;
            break;
//////////////////////////////////////////////////////////////////////
// ERROR!!!!
        default: break;
        }// END SWITCH
    }// END WHILE URXDA (Data Available)
    IFS0bits.U1RXIF = 0; // RESET Interrupt FLAG HERE, buffer should be empty!
}// END U1 RX Interrupt


/****************************************
 * UART1 Buffer parser for SACT Protocol
 ***************************************/
void U2_SACT_Parser(void)
{
	while(u2bufhead != u2buftail)
	{
    	u2prev = u2temp;
    	u2temp = u2tmpbuf[u2buftail++];
        switch(SACT_state) {
//////////////////////////////////////////////////////////////////////
// NO SYNC STATE
            case SACT_NOSYNC:
                rx2buf[rx2cnt++]=u2temp;
                switch(u2temp) {
                    case CR : SACT_flags.cr_rec2 = 1;break;
                    case LF :
                        if (u2prev == CR) {
                            // TERMINATE STRING
                            rx2buf[rx2cnt]=NULLC;
                            process_SYNC_U2();
                            rx2cnt = 0;
                            SACT_flags.cr_rec2 = 0;
                        }
                        else
                        {
                            putsUART((unsigned char*)CRLFMsg,&UART2);
                            rx2cnt = 0;
                        }
                        break;
                    default : //rx2buf[rx2cnt++]=u2temp;
                        if(rx2cnt > (MAX_ASCIILEN-1)) {
                            putsUART((unsigned char*)SyncMsg,&UART2);
                            rx2cnt = 0;
                        }
                        if(SACT_flags.cr_rec2) {
                            putsUART((unsigned char*)CRLFMsg,&UART2);
                            rx2cnt = 0;
                            SACT_flags.cr_rec2 = 0;
                        }
                        break;
                }
                break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART2
            case SACT_ASCII_U2:
                rx2buf[rx2cnt++]=u2temp;
                switch(u2temp) {
                    case CR : SACT_flags.cr_rec2 = 1;break;
                    case LF :
                        if (u2prev == CR) {
                            // TERMINATE STRING
                            rx2buf[rx2cnt]=NULLC;
                            process_ASCII(rx2buf,rx2cnt,&UART2);
                            rx2cnt = 0;
                            SACT_flags.cr_rec2 = 0;
                        }
                        else
                        {
                            putsUART((unsigned char*)CRLFMsg,&UART2);
                            rx2cnt = 0;
                        }
                        break;
                    default : //rx2buf[rx2cnt++]=u2temp;
                        if(rx2cnt > (MAX_ASCIILEN-1))
                        {
                            putsUART((unsigned char*)ErrorMsg,&UART2);
                            rx2cnt = 0;
                        }
                        if(SACT_flags.cr_rec2) {
                            putsUART((unsigned char*)CRLFMsg,&UART2);
                            rx2cnt = 0;
                            SACT_flags.cr_rec2 = 0;
                        }
                        break;
                }
                break;
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART2
            case SACT_BIN_U2:
                rx2buf[rx2cnt++]=u2temp;
                process_BIN(rx2buf,rx2cnt);
                if(SACT_flags.valid_idx) {
                    rx2cnt = 0; //Reset count if a command has been processed!
                    SACT_flags.valid_idx = 0;
                }
                break;
//////////////////////////////////////////////////////////////////////
// ASCII MODE ON UART1, NOT MANAGED HERE (see U1RXInterrupt)
            case SACT_ASCII_U1:
//////////////////////////////////////////////////////////////////////
// BINARY MODE ON UART1, NOT MANAGED HERE (see U1RXInterrupt)
            case SACT_BIN_U1: U2TXREG = u2temp;
                rx2cnt=0;
                break;
//////////////////////////////////////////////////////////////////////
// ERROR!!!!
            default: break;
        }// END SWITCH
    } // END WHILE Data Available

}// END U2 SACT Parser

/****************************************
 * function to detect SYNC on UART1
 ***************************************/
void process_SYNC_U1(void)
{
    int16_t cmpres;
    switch(SYNC_U1_step)
    {
        case 0: cmpres = memcmp(rx1buf,"SYNC0",5);
                if((cmpres == 0)&&(rx1cnt == 7))
                {
                    SYNC_U1_step++;
                    putsUART(rx1buf,&UART1);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART1);
                }
                break;
        case 1: cmpres = memcmp(rx1buf,"SYNC1",5);
                if((cmpres == 0)&&(rx1cnt == 7))
                {
                    SYNC_U1_step++;
                    putsUART(rx1buf,&UART1);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART1);
                    SYNC_U1_step = 0;
                }    
                break;
        case 2: cmpres = memcmp(rx1buf,"SYNC",4);
                if((cmpres == 0)&&(rx1cnt == 7))
                {
                    if (rx1buf[4]=='A')
                    {
                        putsUART(rx1buf,&UART1);
                        SACT_state = SACT_ASCII_U1;
                    }    
                    else if(rx1buf[4]=='B')
                        {
                            putsUART(rx1buf,&UART1);
                            SACT_state = SACT_BIN_U1;
                        }
                }
                
                if(SACT_state == SACT_NOSYNC) putsUART((unsigned char*)"NO SYNC!\r\n",&UART1);
                
                SYNC_U1_step = 0;    
                break;
        default: break;
    }// END switch SYNC_.._step    
}//END process_SYNC

/****************************************
 * function to detect SYNC on UART2
 ***************************************/
void process_SYNC_U2(void)
{
    int16_t cmpres;
    
    switch(SYNC_U2_step)
    {
        case 0: cmpres = memcmp(rx2buf,"SYNC0",5);
                if((cmpres == 0)&&(rx2cnt == 7))
                {
                    SYNC_U2_step++;
                    putsUART(rx2buf,&UART2);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART2);
                }
                break;
        case 1: cmpres = memcmp(rx2buf,"SYNC1",5);
                if((cmpres == 0)&&(rx2cnt == 7))
                {
                    SYNC_U2_step++;
                    putsUART(rx2buf,&UART2);
                }
                else
                {
                    putsUART((unsigned char*)"NO SYNC!\r\n",&UART2);
                    SYNC_U2_step = 0;
                }    
                break;
        case 2: cmpres = memcmp(rx2buf,"SYNC",4);
                if((cmpres == 0)&&(rx2cnt == 7))
                {
                    if (rx2buf[4]=='A')
                    {
                        putsUART(rx2buf,&UART2);
                        SACT_state = SACT_ASCII_U2;
                    }    
                    else if(rx2buf[4]=='B')
                        {
                            putsUART(rx2buf,&UART2);
                            SACT_state = SACT_BIN_U2;
                        }
                }
                
                if(SACT_state == SACT_NOSYNC) putsUART((unsigned char*)"NO SYNC!\r\n",&UART2);
                
                SYNC_U2_step = 0;    
                break;
        default: break;
    }// END switch SYNC_.._step    
}//END process_SYNC
/******************************************************************************
 * functions to manage ASCII mode protocol:
 * - process_ASCII: calls parse functions on the whole packet
 * - CheckHelp: checks if a help request is received
 * - GetMsgIndex: scans the table of valid commands to identify index
 *                corresponding to received one
 * - GetParamASCII: shows to UART the value of requested parameter
 *****************************************************************************/
void process_ASCII(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg)
{
    uint8_t idx = 0; //indice
    uint8_t argcount = 0; //contatore argomenti
    uint8_t count = 0; //contatotore
    uint8_t accum = 0; //accumulatore
    int16_t args[MAXARGS]; //vettore degli argomenti
    int32_t temparg; //argomento temporaneo
    unsigned char tempstr[8]; //ONLY INT VAL EXPECTED //stringa temporanea

    CheckHelp(rxbuf,rxcnt,ureg);
    if(!SACT_flags.help_req)
    {
        idx = GetMsgIndex(rxbuf);
    }
    else
    {
        SACT_flags.help_req = 0;
        return;
    }
    
    if(SACT_flags.valid_idx)
    {    
        //RESET valid_idx flag
        SACT_flags.valid_idx = 0;
        
        //received COMMAND is short, it is a a "GET param", a command without args
        //OR an invalid command 
        if((rxbuf[3] != SPACE)||(rxcnt < 5 + command_data[idx].args*2))
        {
            if(idx >= N_COMMANDS)
            {
                //Command is a parameter request, without args -> GET Param!!
                GetParamASCII(idx,ureg);
                return;
            }
            else if(command_data[idx].args != 0) //received command requires NO args
                {
                    putsUART((unsigned char*)ErrorParaMsg,ureg); //Wrong number of arguments
                                                                 //TOO FEW
                    return;
                }
        }
        
        //SOMETHING THAT SEEMS GOOD in the buffer..
        while(argcount < command_data[idx].args)
        {    
            // search for a space or CR
            // if we are here we should have at least a space or a CR in the buffer
            // from index 4 to ..
            while((rxbuf[count+accum+4] != SPACE)&&(rxbuf[count+accum+4] != CR)) count++;
            
            if(count < 8) //ONLY INT VAL EXPECTED
            {
                memcpy(tempstr,&rxbuf[accum+4],count);
                // TERMINATE STRING
                tempstr[count] = NULLC;
                temparg = atol((char *)tempstr);
                // CHECK BOUNDS
                if((temparg < INT_MIN) || (temparg > INT_MAX))
                {
                    putsUART((unsigned char*)ErrorParaLimitMsg,ureg);
                    return;
                }

                args[argcount] = (int16_t)temparg;
                accum += count+1; //discard space or CR
                count = 0;
            }
            else
            {
                putsUART((unsigned char*)ErrorParaLengthMsg,ureg); //args too long
                return;
            }

            count = 0;
            argcount++;

            //CHECK N.ARGS
            if((rxbuf[accum+4] == LF)&&(argcount < command_data[idx].args))
                {
                    putsUART((unsigned char*)ErrorParaMsg,ureg); //Wrong number of arguments
                                                                 //TOO FEW
                    return;
                }
        }//END while argcount

        //CHECK N.ARGS
        if((accum + 4) != (rxcnt-1))
                {
                    putsUART((unsigned char*)ErrorParaMsg,ureg); //Wrong number of arguments
                                                                 //TOO MUCH
                    return;
                }
		
		//ECHO COMMAND
		putsUART(rxbuf,ureg);
        //NOW WE HAVE VALID COMMAND AND VALID ARGUMENTS!!!
        ExecCommand(idx,args);
        
        //ERROR IN A PARAMETER VALUE
        if(SACT_flags.param_limit)
            {
                putsUART((unsigned char*)ErrorParaLimitMsg,ureg);
                SACT_flags.param_limit = 0;
            }

        //COMMAND NOT CONSISTENT WITH CONTROL MODE
        if(SACT_flags.wrong_mode)
            {
                putsUART((unsigned char*)ErrorControlModeMsg,ureg);
                putsUART((unsigned char*)ControlModeMsg[control_mode.state],ureg);
                SACT_flags.wrong_mode = 0;
            }

        //ECHO COMMAND???
        //putsUART(rxbuf,ureg);

    }//END if valid_idx
    else
    {
        //remplace ?? con rxbuf
        putsUART((unsigned char*)ErrorMsg,ureg); //invalid command
    }
    
}//END process_ASCII

// GET MESSAGE INDEX
unsigned char GetMsgIndex(unsigned char *rxbuf)
{
    uint8_t tempidx = 0;
    int16_t cmpres;
        
    while(tempidx < (N_COMMANDS + N_PARAMS))
    {
        cmpres = memcmp(rxbuf,command_data[tempidx].quick_msg,3);
        if(cmpres == 0)
        {
            SACT_flags.valid_idx = 1;
            return tempidx;
        }
            
        tempidx++;
    }
    if(tempidx == (N_COMMANDS + N_PARAMS))    return 0xFF;

    return 0xFE; // should never get here, just to suppress warning
}//END GetMsgIndex

// GET PARAMETER in ASCII MODE
void GetParamASCII(uint8_t idx, volatile UART *ureg)
{
    putsUART((unsigned char*)command_data[idx].line1_msg,ureg);
    putcUART(HT,ureg);
    putuiUART(parameters_RAM[idx-N_COMMANDS],ureg);
    putcUART(CR,ureg);putcUART(LF,ureg);
}

// Check if a request for help is received
void CheckHelp(unsigned char *rxbuf, uint8_t rxcnt,volatile UART *ureg)
{
  uint8_t idx = 0;
    
  
    if (rxbuf[0] == '?')
    {
      if(rxcnt == 4)
          {
        switch (rxbuf[1])
        {
        case '?':   do
                    putsUART((unsigned char *)HelpMsg_data[idx],ureg);
                    while (++idx < MAX_HELPMSG);
                    break;

        case 'M':   putsUART((unsigned char *)MotorParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(MOTORPARA,ureg);
                    break;

        case 'R':   putsUART((unsigned char *)RobotParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(ROBOTPARA,ureg);
                    break;

        case 'C':   putsUART((unsigned char *)ControlParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(CONTROLPARA,ureg);
                    break;

        case 'I':   putsUART((unsigned char *)IOParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader,ureg);
                    SendHelpInfo(HWIOPARA,ureg);
                    break;

        case 'A':   putsUART((unsigned char *)CommandParaHeader,ureg);
                    putsUART((unsigned char *)ParaHeader_cmd,ureg);
                    SendHelpInfo(COMMANDPARA,ureg);
                    break;

        default:    putsUART((unsigned char*)ErrorMsg,ureg);
                    break;
        }//END switch
       
       } // END if rxcnt
      else
      {
         putsUART((unsigned char*)ErrorMsg,ureg);
      }

    // SET that a help req is received
    SACT_flags.help_req = 1;

  }//END if rxbuf[0]
}//END CheckHelp

// SEND HELP AND PARAMS searching in the commanda_data table
// for matching indexes
void SendHelpInfo(uint8_t table,volatile UART *ureg)
{

uint8_t idx = 0;
uint8_t count = 0;

do
    if (idx == help_info[table][count])
        {
        putsUART((unsigned char *)command_data[idx].line1_msg,ureg);
        putcUART(HT,ureg);putcUART(HT,ureg);
        putsUART((unsigned char *)command_data[idx].quick_msg,ureg);
        putcUART(HT,ureg);putcUART(HT,ureg);
        if(table == COMMANDPARA)
            putuiUART(command_data[idx].args,ureg); // Send Args
        else
            putuiUART(parameters_RAM[idx-N_COMMANDS],ureg); // SendVALUE
        putcUART(CR,ureg);putcUART(LF,ureg);
        count++;
        } 
while (++idx < (N_COMMANDS + N_PARAMS));
}

/******************************************************************************
 * functions to manage BINARY mode protocol:
 * - process_BIN: detects header and counts bytes 
 * - ParseBINCommand: parse the data packet
 * - GetParamBIN: sends current value of a parameter
 *****************************************************************************/
void process_BIN(unsigned char *rxbuf, uint8_t rxcnt)
{
    if((!SACT_flags.valid_header) && (rxcnt>1))								//verifica headers
    {
        if((rxbuf[rxcnt-2]==SACT_HEAD1)&&(rxbuf[rxcnt-1]==SACT_HEAD2)) SACT_flags.valid_header = 1;
        return;
    }
    
    if(SACT_flags.valid_header && !SACT_flags.packet_full)
    {
        BINRXbuf[BINRXcnt++] = rxbuf[rxcnt-1]; //rxcnt-1 because if we are here it is the latest char..
        if(BINRXcnt == (BINRXbuf[0]+1)) //BINRXbuf includes byte count itself
        {
            if(BINRXbuf[BINRXcnt-1] == SACT_EOP)
            {
                SACT_flags.packet_full = 1;
            }
            else 
            {
                SACT_flags.packet_full = 0;
                SACT_flags.valid_header = 0;
                return;
            }
        }
    }
    
    // NOT IN THE ELSE.. must check immediately!
    if(SACT_flags.packet_full)
    {
        ParseBINCommand();
        BINRXcnt=0;
        SACT_flags.packet_full = 0;
        SACT_flags.valid_header = 0;
        SACT_flags.valid_idx = 1;
    }
}

// CALCULATES CRC AND UNPACK COMMAND/ARGS
void ParseBINCommand(void)
{
    
    uint8_t idx = 0;
    uint8_t argcount = 0;
    uint8_t count = 0;
    uint8_t accum = 0;
    int16_t args[MAXARGS];
    WRD temparg;
    
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;

    while(count < (BINRXbuf[0]-3)) //BINRXbuf includes byte count itself at 0 index
    {
        crc_16 = update_crc_16(crc_16,BINRXbuf[count+1]);
        count++;
    }

    temparg.ui = crc_16;

    if((temparg.uc[0] == BINRXbuf[BINRXcnt-3]) && (temparg.uc[1] == BINRXbuf[BINRXcnt-2]))
    {
        idx = BINRXbuf[1]; //BINRXbuf includes byte count itself
                           //index 1 is the command id
        
        // IF COMMAND IS A PARAMETER NUMBER WITHOUT ARGS
        // IT IS A 'GET' REQUEST           
        if((idx >= N_COMMANDS)&&(BINRXcnt==5))
        {
            if(SACT_state == SACT_BIN_U1)
                GetParamBIN(idx,&UART1);
            else //if we are here we have certainly SACT_BIN_U2
                GetParamBIN(idx,&UART2);
            return;
        }
        else if(idx >= (N_PARAMS+N_COMMANDS))
        {
            //INVALID COMMAND!
            status_flags.comm_err_bad_cmd = 1;
            return;
        }
        
        while(argcount < command_data[idx].args)
        {
            temparg.uc[0] = BINRXbuf[2+accum];
            temparg.uc[1] = BINRXbuf[3+accum];
            args[argcount] = temparg.i;
            argcount++;
            accum += 2;
            
            if((accum == BINRXcnt-4)&&(argcount < command_data[idx].args))
            {
                //Wrong number of params (TOO FEW)
                status_flags.comm_err_parm_num = 1;
                return;
            }
        }

        if(accum != (BINRXbuf[0]-4))
        {
            //Wrong number of params (TOO FEW)
            status_flags.comm_err_parm_num = 1;
            return;
        }

        //NOW WE HAVE VALID COMMAND AND VALID ARGUMENTS!!!
        ExecCommand(idx,args);

        BINLastCommand = idx;
        
        //ERROR IN A PARAMETER VALUE
        if(SACT_flags.param_limit)
            {
                //Update status flag
                SACT_flags.param_limit = 0;
                status_flags.comm_err_parm_range = 1;
            }

        //COMMAND NOT CONSISTENT WITH CONTROL MODE
        if(SACT_flags.wrong_mode)
            {
                //Update status flag
                SACT_flags.wrong_mode = 0;
                status_flags.comm_err_wrong_mode = 1;
            }
    }//END if crc OK
    else
    {
        //BAD CRC
        status_flags.comm_err_CRC = 1;
    }
}//END ParseBINCommand()

// GET PARAMETER VALUE IN BINARY MODE
void GetParamBIN(uint8_t idx, volatile UART *ureg)
{
    WRD temp;
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;
    uint8_t count = 0;
    
////////PREPARE CONSTANT PART
        BINTXbuf[0] = SACT_HEAD1;
        BINTXbuf[1] = SACT_HEAD2;
        BINTXbuf[2] = 7;
        BINTXbuf[3] = SACT_SPP;
        BINTXbuf[4] = idx;
    
///////PARAM VALUE
        temp.ui = parameters_RAM[idx-N_COMMANDS];
        BINTXbuf[5] = temp.uc[0];
        BINTXbuf[6] = temp.uc[1];
        
////////CALCULATE CRC    
        while(count < 4) 
        {
            crc_16 = update_crc_16(crc_16,BINTXbuf[count+3]);
            count++;
        }

        temp.ui = crc_16;
        BINTXbuf[7] = temp.uc[0];
        BINTXbuf[8] = temp.uc[1];

////////END OF PACKET
        BINTXbuf[9] = SACT_EOP;

////////SEND PACKET
        if(SACT_state == SACT_BIN_U1)
        {
            SendNUART(BINTXbuf,&UART1,10);
        }
        else //if we are here we have certainly SACT_BIN_U2
        {
            SendNUART(BINTXbuf,&UART2,10);
        }                        
}//END GetParamBIN

/******************************************************************************
 * CORE FUNCTION for commands execution
 *****************************************************************************/

//Rigestione di tutti i commandi  vengono passati indice del commando e argomenti
void ExecCommand(uint8_t idx,int16_t *args)
{
    int16_t temp[N_MOTOR],i;
    //char t[2]="";
    if(idx >= N_COMMANDS) //Da indice 11 al 29
    { // IT IS A PARAMETER UPDATE REQUEST
        if(control_mode.state == OFF_MODE)//deve essere spento
        {
            if((args[0] >= command_data[idx].min)&&(args[0] <= command_data[idx].max))
                {
                    parameters_RAM[idx-N_COMMANDS] = args[0];
                    control_flags.PAR_update_req = 1;
                }    
            else
                SACT_flags.param_limit = 1;
        }
        else
            SACT_flags.wrong_mode = 1;
    }
    else
    { // IT IS AN ACTION COMMAND    ---devo cominciare a vedere da qua

        switch(idx)
        {
            case 0: // DISCONNECT
                    SACT_state = 0;
                    control_mode.off_mode_req = 1;
                     break;
            case 1: // CONTROL MODE "CMO arg" 
                    if(control_mode.state == OFF_MODE) {
                        /*putsUART((unsigned char *) "switching in ", &UART1);
                        t[0]=49+args[0];
                        putsUART((unsigned char *) t, &UART1);
                        putsUART((unsigned char *) " mode\n", &UART1);*/
                        switch(args[0]) {
                            case OFF_MODE: break; //non faccio nulla
                            case TORQUE_MODE: control_mode.torque_mode_req = 1;break;
                            case AX_POS_MODE  : control_mode.ax_pos_mode_req = 1;break;
                            case CART_MODE    : control_mode.cart_mode_req = 1;break;
                            case TRACK_MODE    : control_mode.track_mode_req = 1;break;
                            default : break; // SHOULD NEVER HAPPEN
                        }//END switch args[0]
                    }
                    else {
                        if(args[0] == OFF_MODE) {
                            control_mode.off_mode_req = 1;
                            SSP_config.word = 0;
                        }
                    }
                    break;
            case 2: // SET TORQUE REF
                    if(control_mode.state == TORQUE_MODE)
                    {
                        //putsUART((unsigned char *) "set current on  ", &UART1);
                        for(i=0;i<N_MOTOR;i++) {
                            temp[i] = args[i];
                            if(temp[i] < 0) temp[i] = -temp[i];
                            //putiUART(temp[i],&UART1);
                            //putsUART((unsigned char *) " max current is ", &UART1);
                            //putiUART(max_current,&UART1);
                            //putsUART((unsigned char *) "\n", &UART1);
                            if(temp[i] > max_current) {//se sono maggiore del massimo
                                SACT_flags.param_limit = 1;
                                break;
                            }
                        }
                        if (SACT_flags.param_limit==0)
                            for(i=0;i<N_MOTOR;i++)
                                    MOTOR[i].rcurrent = args[i];
                    }
                    else 
                         SACT_flags.wrong_mode = 1;
                    break;
            case 3: // SET AXIS POSITION REFS AND TRACK MODE REFS
                    if((control_mode.state == AX_POS_MODE)||(control_mode.state == TRACK_MODE))
                    {
                        for (i=0;i<N_MOTOR;i++) {
                            angleJoints_temp[i] = convert_decdeg_to_rad(args[i]);
                        }
                        status = delta_calcForward(angleJoints_temp, &coordinates_temp);
                        if (status == 0)
                            status = joints_accessible_angle(angleJoints_temp);
                        if (status == 0)
                            status = joints_accessible_pos(coordinates_temp.x, coordinates_temp.y, coordinates_temp.z);
                        //SSP_config.word = 1;
                        switch(status) {
                            case 0: //OK
                                move(angleJoints_temp);
                                break;
                            case -1://point out of space
                                SACT_flags.wrong_mode = 1;
                                putsUART((unsigned char*)ErrorSpace,&UART1);
                                SACT_flags.wrong_mode =0 ;
                                break;
                            case -3://danger joints
                                SACT_flags.wrong_mode = 1;
                                putsUART((unsigned char*)ErrorJoints,&UART1);
                                SACT_flags.wrong_mode = 0;
                                break;
                        }
                    }
                    else
                        SACT_flags.wrong_mode = 1;
                    break;
            case 4: // SET POINT FOR CARTESIAN MODE
                    if(control_mode.state == CART_MODE)
                    {
                        coordinates_temp.x= convert_decmill_to_meters(args[0]);
                        coordinates_temp.y = convert_decmill_to_meters(args[1]);
                        coordinates_temp.z = convert_decmill_to_meters(args[2]);
                        status = delta_calcInverse(angleJoints_temp, &coordinates_temp);
                        if (status == 0) status = joints_accessible_angle(angleJoints_temp);
			if (status == 0) status = joints_accessible_pos(coordinates_temp.y,coordinates_temp.y*cos120-coordinates_temp.x*sin120, coordinates_temp.y*cos120+coordinates_temp.x*sin120);
                        //SSP_config.word = 1;
                        switch(status) {
                            case 0: // OK
                                move(angleJoints_temp);
                                break;
                            case -1://point out of space
                                SACT_flags.wrong_mode = 1;
                                putsUART((unsigned char*)ErrorSpace,&UART1);
                                SACT_flags.wrong_mode = 0;
                                break;
                            case -3://joints error
                                SACT_flags.wrong_mode = 1;
                                putsUART((unsigned char*)ErrorJoints,&UART1);
                                SACT_flags.wrong_mode = 0;
                                break;
                        }
                    }
                    else
                        SACT_flags.wrong_mode = 1;
                    break;
            case 5: //MOVE INCREMENTAL
                if (control_mode.state == CART_MODE) {
                    coordinates_temp.x = coordinates_actual.x + convert_decmill_to_meters(args[0]);
                    coordinates_temp.y = coordinates_actual.y + convert_decmill_to_meters(args[1]);
                    coordinates_temp.y = coordinates_actual.y + convert_decmill_to_meters(args[1]);
                    status = delta_calcInverse(angleJoints_temp, &coordinates_temp);
                    if (status == 0) status = joints_accessible_angle(angleJoints_temp);
                    if (status == 0) status = joints_accessible_pos(coordinates_temp.y,coordinates_temp.y*cos120-coordinates_temp.x*sin120, coordinates_temp.y*cos120+coordinates_temp.x*sin120);
                    //SSP_config.word = 1;
                    switch(status) {
                        case 0: // OK
                            move(angleJoints_temp);
                            break;
                        case -1://point out of space
                            SACT_flags.wrong_mode = 1;
                            putsUART((unsigned char*)ErrorSpace,&UART1);
                            SACT_flags.wrong_mode = 0;
                            break;
                        case -3://joints error
                            SACT_flags.wrong_mode = 1;
                            putsUART((unsigned char*)ErrorJoints,&UART1);
                            SACT_flags.wrong_mode = 0;
                            break;
                    }
                }
                else
                    SACT_flags.wrong_mode = 1;
                break;
            case 6:	 //HOME
                if (control_mode.state == AX_POS_MODE)
                {
                    //SSP_config.word = 1;
                    //-------------------------------
                    home_f.state = 0;
                    home_f.done = 0;
                    home_f.homing_active = 1;
                    //-------------------------------
                }
                else
                    SACT_flags.wrong_mode = 1;
                break;
            case 7: //GRIP
                break;
            case 8: // PULSE
                Nop();
                break;
            case 9: // UPDATE EEPROM;
                putsUART((unsigned char *)"performing UEE\r\n",&UART1);
                //int i, addr = 0x0000;
                //int value;
                DataEEInit();
                if(control_mode.state == OFF_MODE)
                    {
                        if(!control_flags.EE_update_req)
                            control_flags.EE_update_req = 1;
                    }
                    else
                        SACT_flags.wrong_mode = 1;
                    break;
            case 10:// SET SSP configuration
                    SSP_config.word = args[0]; 
                    break;
            default : break;
            
        }//END switch idx for action commands
         
        // CAN SAFELY RESET TIMEOUT (done all checks on commands and args)
        SACT_flags.timeout = 0;
    }
}//END ExecCommand
/******************************************************************************
 * TIMEOUT MANAGEMENT (public, called by slow_event_handler)
 *****************************************************************************/
void SACT_timeout(void)
{
    if((SACT_state == SACT_BIN_U1)||(SACT_state == SACT_BIN_U2))
    {
        SACT_flags.timeout++;
        if(SACT_flags.timeout > SACT_TIME_LIMIT/SLOW_RATE)
        {
            SACT_flags.timeout = 0;
            SACT_state = SACT_NOSYNC;
            control_mode.off_mode_req = 1;
        }    
    }
}

/******************************************************************************
 * SENDS Sabot Sensor Packet (public, called by slow_event_handler)
 *****************************************************************************/
void SACT_SendSSP(void)
{
    unsigned DIR[3]={DIR1,DIR2,DIR3};
    char t[2]="",coords[]="xyz";
    volatile UART *ureg;
    WRD temp;
    LNG templong;
    uint8_t accum = 0;
    uint8_t count = 0,i,j;
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;
    int16_t temp2[N_MOTOR];
    
    if(SSP_config.word != 0)
    {
////////////////////////////////////////////////////////////////////////
////////IF SACT state ASCII send human readable data
        if((SACT_state == SACT_ASCII_U1)||(SACT_state == SACT_ASCII_U2))
        {//ASCII MODE
        if(SACT_state == SACT_ASCII_U1)
            ureg = &UART1;
        else //if we are here we have certainly SACT_BIN_U2
            ureg = &UART2;

        /*temp.i = SSP_config.word;
        putuiUART(temp.ui,ureg);
        putcUART(HT,ureg);*/

/////////SENSOR DATA
        if(SSP_config.encoders)
        {
            for (i=0;i<N_MOTOR;i++) {
                templong.l = MOTOR[i].mposition;
                 putuiUART(templong.ui[1],ureg);
                //putcUART(VL,ureg);
                //putcUART(SC,ureg);
                putuiUART(templong.ui[0],ureg);
                putcUART(HT,ureg);
            }
        }// END if encoders

        if((SSP_config.cartesian))
        {
            temp2[0] = (int16_t) convert_meters_to_decmill(coordinates_actual.x);
            temp2[1] = (int16_t) convert_meters_to_decmill(coordinates_actual.y);
            temp2[2] = (int16_t) convert_meters_to_decmill(coordinates_actual.z);
            for (i=0;i<N_MOTOR;i++) {
                putcUART(HT,ureg);
                t[0]=coords[i];
                putsUART((unsigned char *)t,ureg);
                putsUART((unsigned char *)": ",ureg);
                putiUART(temp2[i],ureg);
                putcUART(VL,ureg);
            }
            putcUART(HT,ureg);
            for (i=0;i<N_MOTOR;i++) {
                temp2[i] = (int16_t) convert_rad_to_decdeg(angleJoints_actual[i]);
                putsUART((unsigned char *)"T",ureg);
                t[0]=(49+i);
                putsUART((unsigned char *)t,ureg);
                putsUART((unsigned char *)": ",ureg);
                putiUART(temp2[i],ureg);
                putcUART(VL,ureg);
                putcUART(HT,ureg);
            }
            for (i=0;i<N_MOTOR;i++) {
                temp.i =TRAJ[i].param.qdPosition * ticks_to_deg*10;
                putsUART((unsigned char *)"R",ureg);
                t[0]=(49+i);
                putsUART((unsigned char *)t,ureg);
                putsUART((unsigned char *)": ",ureg);
                putiUART(temp.i,ureg);
                putcUART(VL,ureg);
                putcUART(HT,ureg);
                temp.i =PID[i].flag.Pos.saturated;
                    putsUART((unsigned char *)"\t psat:",ureg);
                    putiUART(temp.i,ureg);
            }
        }// END if cartesian
        
        if(SSP_config.analogs)
        {
                        
        }// END if analogs

        if(SSP_config.digitals)
        {
        }// END if digitals

        if(SSP_config.sonars)
        {
            
        }// END if sonars

        if(SSP_config.currents)
        {
            for(i=0;i<N_MOTOR;i++) {
                //if(DIR[i])  {
                    //
                    temp.i = MOTOR[i].mcurrent;
                    putsUART((unsigned char *)"\t m",ureg);
                    t[0]=(49+i);
                    putsUART((unsigned char *)t,ureg);
                    putsUART((unsigned char *)": ",ureg);
                    putiUART(temp.i,ureg);
                    putsUART((unsigned char *)"\t r",ureg);
                    putsUART((unsigned char *)t,ureg);
                    putsUART((unsigned char *)": ",ureg);
                    temp.i = MOTOR[i].rcurrent;
                    putiUART(temp.i,ureg);
                    putsUART((unsigned char *)"\t mf",ureg);
                    putsUART((unsigned char *)t,ureg);
                    putsUART((unsigned char *)": ",ureg);
                    temp.i = MOTOR[i].mcurrent_filt;
                    putiUART(temp.i,ureg);
                    temp.i =PID[i].flag.Current.saturated;
                    putsUART((unsigned char *)"\t csat:",ureg);
                    putiUART(temp.i,ureg);
                    putsUART((unsigned char *)"\n",ureg);
            }
        }// END if currents

        if(SSP_config.wheel_vel)
        {
            temp.i = MOTOR[0].mvelocity;
            putiUART(temp.i,ureg);
            putcUART(HT,ureg);
			putcUART(HT,ureg);
            
            temp.i = MOTOR[1].mvelocity;
            putiUART(temp.i,ureg);
            putcUART(HT,ureg);
			putcUART(HT,ureg);

			putsUART((unsigned char *)"velocity1RPM: ",ureg);
			temp.i = MOTOR[0].velocityRPM;
            putiUART(temp.i,ureg);
            putcUART(HT,ureg);

			putsUART((unsigned char *)"velocity2RPM: ",ureg);
			temp.i = MOTOR[1].velocityRPM;
            putiUART(temp.i,ureg);
            putcUART(HT,ureg);
        }// END if wheel vel

        if(SSP_config.linrot_vel)
        {
            
        }// END if linear/rot. vel
////////ALL DATA SENT IN ASCII MODE
        putcUART(CR,ureg);putcUART(LF,ureg);        
            
        }// END if SACT_state ASCII..
            /////////////////////////////////////////////////////////////////////////
            ////////IF SACT state BIN send binary data according to SACT protocol
        else if ((SACT_state == SACT_BIN_U1) || (SACT_state == SACT_BIN_U2)) { //BINARY MODE
            if (SACT_state == SACT_BIN_U1)
                ureg = &UART1;
            else //if we are here we have certainly SACT_BIN_U2
                ureg = &UART2;

            ////////PREPARE CONSTANT PART
            BINTXbuf[0] = SACT_HEAD1;
            BINTXbuf[1] = SACT_HEAD2;
            BINTXbuf[3] = SACT_SSP;
            temp.i = SSP_config.word;
            BINTXbuf[4] = temp.uc[0];
            BINTXbuf[5] = temp.uc[1];

            ////////PREPARE SENSOR DATA
            if (SSP_config.encoders) {
                for (i = 0; i < N_MOTOR; i++) {
                    templong.l = MOTOR[i].mposition;
                    for (j = 0; j < 4; j++, accum++)
                        BINTXbuf[accum + 6] = templong.uc[j];
                }
            }// END if encoders

            if (SSP_config.cartesian) {
                templong.l = (int32_t) convert_meters_to_decmill(coordinates_actual.x);
                for (j = 0; j < 4; j++, accum++)
                    BINTXbuf[accum + 6] = templong.uc[j];
                templong.l = (int32_t) convert_meters_to_decmill(coordinates_actual.y);
                for (j = 0; j < 4; j++, accum++)
                    BINTXbuf[accum + 6] = templong.uc[j];
                templong.l = (int32_t) convert_meters_to_decmill(coordinates_actual.z);
                for (j = 0; j < 4; j++, accum++)
                    BINTXbuf[accum + 6] = templong.uc[j];
            }// END if odometry

            if (SSP_config.analogs) {


            }// END if analogs

            if (SSP_config.digitals) {

            }// END if digitals

            if (SSP_config.sonars) {

            }// END if sonars

            if (SSP_config.currents) {
                for (i = 0; i < N_MOTOR; i++) {
                    if (DIR[i])
                        temp.i = -MOTOR[i].mcurrent_filt;
                    else
                        temp.i = MOTOR[i].mcurrent_filt;
                    BINTXbuf[accum + 6] = temp.uc[0];
                    accum++;
                    BINTXbuf[accum + 6] = temp.uc[1];
                    accum++;
                }
            }// END if currents

            if (SSP_config.wheel_vel) {

            }// END if wheel vel

            if (SSP_config.linrot_vel) {

            }// END if linear/rot. vel

            ////////SENSOR DATA PREPARED, proceed with rest
            BINTXbuf[2] = accum + 6; //BYTE COUNT

            ////////CALCULATE CRC
            while (count < (accum + 3)) {
                crc_16 = update_crc_16(crc_16, BINTXbuf[count + 3]);
                count++;
            }

            temp.ui = crc_16;
            BINTXbuf[accum + 6] = temp.uc[0];
            BINTXbuf[accum + 7] = temp.uc[1];

            ////////END OF PACKET
            BINTXbuf[accum + 8] = SACT_EOP;

            ////////SEND PACKET
            SendNUART(BINTXbuf, ureg, accum + 9);

        } // END else if SACT_state BIN..
    }//END if config.word != 0
}//END SACT_SenSSP

/******************************************************************************
 * SENDS Sabot Diagnostic Packet (public, called by slow_event_handler)
 *****************************************************************************/
void SACT_SendSDP(void) {
    static t_status_flags status_flags_prev;

    LNG temp;
    uint8_t count = 0;
    // to be compatible with lib_crc, u.short corresponds
    // to an u.int (16 bit) in MPLAB C30
    unsigned short crc_16 = 0;

    // RESET COMM errors
    if (SACT_state == SACT_NOSYNC)
        status_flags.dword = status_flags.dword & 0xFF00FFFF;

    ////IF SACT state ASCII send human readable info about error
    if ((SACT_state == SACT_ASCII_U1) || (SACT_state == SACT_ASCII_U2)) {
        if (((status_flags.dword & 0x000000FF) != (status_flags_prev.dword & 0x000000FF)) && ((status_flags.dword & 0x000000FF) != 0)) {
            // TODO: better fault messages..
            if (SACT_state == SACT_ASCII_U1)
                putsUART((unsigned char*) "FAULT DETECTED!\r\n", &UART1);
            else
                putsUART((unsigned char*) "FAULT DETECTED!\r\n", &UART2);
        }
        status_flags_prev.dword = status_flags.dword;

    } else if ((SACT_state == SACT_BIN_U1) || (SACT_state == SACT_BIN_U2)) {
        ////////PREPARE CONSTANT PART
        BINTXbuf[0] = SACT_HEAD1;
        BINTXbuf[1] = SACT_HEAD2;
        BINTXbuf[2] = 13;
        BINTXbuf[3] = SACT_SDP;

        ////////PREPARE DIAG data
        temp.ul = status_flags.dword;
        BINTXbuf[4] = temp.uc[0];
        BINTXbuf[5] = temp.uc[1];
        BINTXbuf[6] = temp.uc[2];
        BINTXbuf[7] = temp.uc[3];

        BINTXbuf[8] = control_mode.state;

        BINTXbuf[9] = BINLastCommand;

        BINTXbuf[10] = 1; // FIRMWARE REVISION v1.4
        BINTXbuf[11] = 4;

#ifdef SIMULATE
        BINTXbuf[12] = 0; // BOARD REVISION X
#endif

#ifdef PROTO_BOARD 
        BINTXbuf[12] = 1; // BOARD REVISION 1
#endif

        ////////CALCULATE CRC
        while (count < 10) {
            crc_16 = update_crc_16(crc_16, BINTXbuf[count + 3]);
            count++;
        }

        temp.ui[0] = crc_16;
        BINTXbuf[13] = temp.uc[0];
        BINTXbuf[14] = temp.uc[1];

        ////////END OF PACKET
        BINTXbuf[15] = SACT_EOP;

        ////////SEND PACKET
        if (SACT_state == SACT_BIN_U1) {
            SendNUART(BINTXbuf, &UART1, 16);
        } else //if we are here we have certainly SACT_BIN_U2
        {
            SendNUART(BINTXbuf, &UART2, 16);
        }
    }//END ELSE if SACT_state BIN..
}//END SACT_SenSSP

