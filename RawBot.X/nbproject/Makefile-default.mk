#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360937237/PWM.o ${OBJECTDIR}/_ext/1360937237/QEI.o ${OBJECTDIR}/_ext/1360937237/Timers.o ${OBJECTDIR}/_ext/1360937237/traps.o ${OBJECTDIR}/_ext/1360937237/ADC_DMA.o ${OBJECTDIR}/_ext/1360937237/PPS.o "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o" "${OBJECTDIR}/_ext/1360937237/Flash Operations.o" ${OBJECTDIR}/_ext/1360937237/InputCapture.o ${OBJECTDIR}/_ext/1360937237/Controls.o ${OBJECTDIR}/_ext/1360937237/PID.o ${OBJECTDIR}/_ext/1360937237/Trajectories.o ${OBJECTDIR}/_ext/1360937237/Kinematix.o ${OBJECTDIR}/_ext/1360937237/motion.o ${OBJECTDIR}/_ext/1360937237/Convert.o ${OBJECTDIR}/_ext/1360937237/Comms.o ${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o ${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o ${OBJECTDIR}/_ext/284212209/Q16wrappers.o ${OBJECTDIR}/_ext/1514861621/lib_crc.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/globals.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360937237/PWM.o.d ${OBJECTDIR}/_ext/1360937237/QEI.o.d ${OBJECTDIR}/_ext/1360937237/Timers.o.d ${OBJECTDIR}/_ext/1360937237/traps.o.d ${OBJECTDIR}/_ext/1360937237/ADC_DMA.o.d ${OBJECTDIR}/_ext/1360937237/PPS.o.d "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o.d" "${OBJECTDIR}/_ext/1360937237/Flash Operations.o.d" ${OBJECTDIR}/_ext/1360937237/InputCapture.o.d ${OBJECTDIR}/_ext/1360937237/Controls.o.d ${OBJECTDIR}/_ext/1360937237/PID.o.d ${OBJECTDIR}/_ext/1360937237/Trajectories.o.d ${OBJECTDIR}/_ext/1360937237/Kinematix.o.d ${OBJECTDIR}/_ext/1360937237/motion.o.d ${OBJECTDIR}/_ext/1360937237/Convert.o.d ${OBJECTDIR}/_ext/1360937237/Comms.o.d ${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o.d ${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o.d ${OBJECTDIR}/_ext/284212209/Q16wrappers.o.d ${OBJECTDIR}/_ext/1514861621/lib_crc.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/globals.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360937237/PWM.o ${OBJECTDIR}/_ext/1360937237/QEI.o ${OBJECTDIR}/_ext/1360937237/Timers.o ${OBJECTDIR}/_ext/1360937237/traps.o ${OBJECTDIR}/_ext/1360937237/ADC_DMA.o ${OBJECTDIR}/_ext/1360937237/PPS.o ${OBJECTDIR}/_ext/1360937237/DEE\ Emulation\ 16-bit.o ${OBJECTDIR}/_ext/1360937237/Flash\ Operations.o ${OBJECTDIR}/_ext/1360937237/InputCapture.o ${OBJECTDIR}/_ext/1360937237/Controls.o ${OBJECTDIR}/_ext/1360937237/PID.o ${OBJECTDIR}/_ext/1360937237/Trajectories.o ${OBJECTDIR}/_ext/1360937237/Kinematix.o ${OBJECTDIR}/_ext/1360937237/motion.o ${OBJECTDIR}/_ext/1360937237/Convert.o ${OBJECTDIR}/_ext/1360937237/Comms.o ${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o ${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o ${OBJECTDIR}/_ext/284212209/Q16wrappers.o ${OBJECTDIR}/_ext/1514861621/lib_crc.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/globals.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ64MC802
MP_LINKER_FILE_OPTION=,--script="../gld/linkerscript.gld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/PWM.o: ../src/PWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PWM.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/PWM.c  -o ${OBJECTDIR}/_ext/1360937237/PWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/PWM.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PWM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/QEI.o: ../src/QEI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/QEI.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/QEI.c  -o ${OBJECTDIR}/_ext/1360937237/QEI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/QEI.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/QEI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Timers.o: ../src/Timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Timers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Timers.c  -o ${OBJECTDIR}/_ext/1360937237/Timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Timers.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Timers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/traps.o: ../src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/traps.c  -o ${OBJECTDIR}/_ext/1360937237/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/traps.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/ADC_DMA.o: ../src/ADC_DMA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/ADC_DMA.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/ADC_DMA.c  -o ${OBJECTDIR}/_ext/1360937237/ADC_DMA.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/ADC_DMA.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mconst-in-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/ADC_DMA.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/PPS.o: ../src/PPS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PPS.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/PPS.c  -o ${OBJECTDIR}/_ext/1360937237/PPS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/PPS.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PPS.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/DEE\ Emulation\ 16-bit.o: ../src/DEE\ Emulation\ 16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/DEE\ Emulation\ 16-bit.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../src/DEE Emulation 16-bit.c"  -o "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -mconst-in-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/InputCapture.o: ../src/InputCapture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/InputCapture.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/InputCapture.c  -o ${OBJECTDIR}/_ext/1360937237/InputCapture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/InputCapture.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/InputCapture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Controls.o: ../src/Controls.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Controls.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Controls.c  -o ${OBJECTDIR}/_ext/1360937237/Controls.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Controls.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Controls.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/PID.o: ../src/PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PID.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/PID.c  -o ${OBJECTDIR}/_ext/1360937237/PID.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/PID.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PID.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Trajectories.o: ../src/Trajectories.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Trajectories.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Trajectories.c  -o ${OBJECTDIR}/_ext/1360937237/Trajectories.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Trajectories.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Trajectories.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Kinematix.o: ../src/Kinematix.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Kinematix.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Kinematix.c  -o ${OBJECTDIR}/_ext/1360937237/Kinematix.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Kinematix.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Kinematix.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/motion.o: ../src/motion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/motion.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/motion.c  -o ${OBJECTDIR}/_ext/1360937237/motion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/motion.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/motion.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Convert.o: ../src/Convert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Convert.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Convert.c  -o ${OBJECTDIR}/_ext/1360937237/Convert.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Convert.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Convert.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Comms.o: ../src/Comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Comms.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Comms.c  -o ${OBJECTDIR}/_ext/1360937237/Comms.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Comms.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Comms.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o: ../src/SACT_Protocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/SACT_Protocol.c  -o ${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o: ../lib_math/FxSqrtAbs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/284212209 
	@${RM} ${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../lib_math/FxSqrtAbs.c  -o ${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/284212209/Q16wrappers.o: ../lib_math/Q16wrappers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/284212209 
	@${RM} ${OBJECTDIR}/_ext/284212209/Q16wrappers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../lib_math/Q16wrappers.c  -o ${OBJECTDIR}/_ext/284212209/Q16wrappers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/284212209/Q16wrappers.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/284212209/Q16wrappers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1514861621/lib_crc.o: ../lib_crc/lib_crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1514861621 
	@${RM} ${OBJECTDIR}/_ext/1514861621/lib_crc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../lib_crc/lib_crc.c  -o ${OBJECTDIR}/_ext/1514861621/lib_crc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1514861621/lib_crc.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1514861621/lib_crc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/main.c  -o ${OBJECTDIR}/_ext/1360937237/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/globals.o: ../src/globals.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/globals.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/globals.c  -o ${OBJECTDIR}/_ext/1360937237/globals.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/globals.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/globals.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1360937237/PWM.o: ../src/PWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PWM.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/PWM.c  -o ${OBJECTDIR}/_ext/1360937237/PWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/PWM.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PWM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/QEI.o: ../src/QEI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/QEI.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/QEI.c  -o ${OBJECTDIR}/_ext/1360937237/QEI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/QEI.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/QEI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Timers.o: ../src/Timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Timers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Timers.c  -o ${OBJECTDIR}/_ext/1360937237/Timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Timers.o.d"        -g -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Timers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/traps.o: ../src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/traps.c  -o ${OBJECTDIR}/_ext/1360937237/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/traps.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/ADC_DMA.o: ../src/ADC_DMA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/ADC_DMA.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/ADC_DMA.c  -o ${OBJECTDIR}/_ext/1360937237/ADC_DMA.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/ADC_DMA.o.d"        -g -omf=elf -mlarge-data -mconst-in-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/ADC_DMA.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/PPS.o: ../src/PPS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PPS.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/PPS.c  -o ${OBJECTDIR}/_ext/1360937237/PPS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/PPS.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PPS.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/DEE\ Emulation\ 16-bit.o: ../src/DEE\ Emulation\ 16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/DEE\ Emulation\ 16-bit.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../src/DEE Emulation 16-bit.c"  -o "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o.d"        -g -omf=elf -mlarge-data -mconst-in-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/DEE Emulation 16-bit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/InputCapture.o: ../src/InputCapture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/InputCapture.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/InputCapture.c  -o ${OBJECTDIR}/_ext/1360937237/InputCapture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/InputCapture.o.d"        -g -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/InputCapture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Controls.o: ../src/Controls.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Controls.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Controls.c  -o ${OBJECTDIR}/_ext/1360937237/Controls.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Controls.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Controls.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/PID.o: ../src/PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PID.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/PID.c  -o ${OBJECTDIR}/_ext/1360937237/PID.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/PID.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PID.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Trajectories.o: ../src/Trajectories.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Trajectories.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Trajectories.c  -o ${OBJECTDIR}/_ext/1360937237/Trajectories.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Trajectories.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Trajectories.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Kinematix.o: ../src/Kinematix.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Kinematix.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Kinematix.c  -o ${OBJECTDIR}/_ext/1360937237/Kinematix.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Kinematix.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Kinematix.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/motion.o: ../src/motion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/motion.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/motion.c  -o ${OBJECTDIR}/_ext/1360937237/motion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/motion.o.d"        -g -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/motion.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Convert.o: ../src/Convert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Convert.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Convert.c  -o ${OBJECTDIR}/_ext/1360937237/Convert.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Convert.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Convert.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/Comms.o: ../src/Comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Comms.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/Comms.c  -o ${OBJECTDIR}/_ext/1360937237/Comms.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/Comms.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Comms.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o: ../src/SACT_Protocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/SACT_Protocol.c  -o ${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/SACT_Protocol.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o: ../lib_math/FxSqrtAbs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/284212209 
	@${RM} ${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../lib_math/FxSqrtAbs.c  -o ${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/284212209/FxSqrtAbs.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/284212209/Q16wrappers.o: ../lib_math/Q16wrappers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/284212209 
	@${RM} ${OBJECTDIR}/_ext/284212209/Q16wrappers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../lib_math/Q16wrappers.c  -o ${OBJECTDIR}/_ext/284212209/Q16wrappers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/284212209/Q16wrappers.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/284212209/Q16wrappers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1514861621/lib_crc.o: ../lib_crc/lib_crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1514861621 
	@${RM} ${OBJECTDIR}/_ext/1514861621/lib_crc.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../lib_crc/lib_crc.c  -o ${OBJECTDIR}/_ext/1514861621/lib_crc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1514861621/lib_crc.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1514861621/lib_crc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/main.c  -o ${OBJECTDIR}/_ext/1360937237/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d"        -g -omf=elf -mlarge-data -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/globals.o: ../src/globals.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/globals.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/globals.c  -o ${OBJECTDIR}/_ext/1360937237/globals.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/globals.o.d"        -g -omf=elf -O0 -I"../include" -I"../lib_math" -I"../lib_crc" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/globals.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/Flash\ Operations.o: ../src/Flash\ Operations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Flash\ Operations.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  "../src/Flash Operations.s"  -o "${OBJECTDIR}/_ext/1360937237/Flash Operations.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -I".." -Wa,-MD,"${OBJECTDIR}/_ext/1360937237/Flash Operations.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Flash Operations.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/1360937237/Flash\ Operations.o: ../src/Flash\ Operations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/Flash\ Operations.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  "../src/Flash Operations.s"  -o "${OBJECTDIR}/_ext/1360937237/Flash Operations.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -I".." -Wa,-MD,"${OBJECTDIR}/_ext/1360937237/Flash Operations.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/Flash Operations.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ../gld/linkerscript.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -Wl,--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="../gld",--no-force-link,--smart-io,-Map="${DISTDIR}/RawBot.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ../gld/linkerscript.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="../gld",--no-force-link,--smart-io,-Map="${DISTDIR}/RawBot.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}/xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/RawBot.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
