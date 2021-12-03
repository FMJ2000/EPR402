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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=aux.c bot.c imu.c master.c oled.c periph.c map.c node.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/aux.o ${OBJECTDIR}/bot.o ${OBJECTDIR}/imu.o ${OBJECTDIR}/master.o ${OBJECTDIR}/oled.o ${OBJECTDIR}/periph.o ${OBJECTDIR}/map.o ${OBJECTDIR}/node.o
POSSIBLE_DEPFILES=${OBJECTDIR}/aux.o.d ${OBJECTDIR}/bot.o.d ${OBJECTDIR}/imu.o.d ${OBJECTDIR}/master.o.d ${OBJECTDIR}/oled.o.d ${OBJECTDIR}/periph.o.d ${OBJECTDIR}/map.o.d ${OBJECTDIR}/node.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/aux.o ${OBJECTDIR}/bot.o ${OBJECTDIR}/imu.o ${OBJECTDIR}/master.o ${OBJECTDIR}/oled.o ${OBJECTDIR}/periph.o ${OBJECTDIR}/map.o ${OBJECTDIR}/node.o

# Source Files
SOURCEFILES=aux.c bot.c imu.c master.c oled.c periph.c map.c node.c



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
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX270F256B
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/aux.o: aux.c  .generated_files/flags/default/af8d663ed5474e2d23c591c8b06a1952ebc3b1a5 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/aux.o.d 
	@${RM} ${OBJECTDIR}/aux.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/aux.o.d" -o ${OBJECTDIR}/aux.o aux.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/bot.o: bot.c  .generated_files/flags/default/4246df63c9e07fb96ba0961cb24309de9223bd0e .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bot.o.d 
	@${RM} ${OBJECTDIR}/bot.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/bot.o.d" -o ${OBJECTDIR}/bot.o bot.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/imu.o: imu.c  .generated_files/flags/default/9b2682d9e0f72342fea9605c1cea9c2a3be43684 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/imu.o.d 
	@${RM} ${OBJECTDIR}/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/imu.o.d" -o ${OBJECTDIR}/imu.o imu.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/master.o: master.c  .generated_files/flags/default/2d6a88431d83f3a5bdcb00aed45f3ccae2299ea5 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/master.o.d 
	@${RM} ${OBJECTDIR}/master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/master.o.d" -o ${OBJECTDIR}/master.o master.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/oled.o: oled.c  .generated_files/flags/default/ebeaae2f204e26d45de61fa6f8a8a7119d534d32 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/oled.o.d 
	@${RM} ${OBJECTDIR}/oled.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/oled.o.d" -o ${OBJECTDIR}/oled.o oled.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/periph.o: periph.c  .generated_files/flags/default/6aeeb676e8150b0d9a8d3b6812042fa0bec3a179 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/periph.o.d 
	@${RM} ${OBJECTDIR}/periph.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/periph.o.d" -o ${OBJECTDIR}/periph.o periph.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/map.o: map.c  .generated_files/flags/default/240950d5877fbd92347c7b1d9bfa0a010f1be19a .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/map.o.d 
	@${RM} ${OBJECTDIR}/map.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/map.o.d" -o ${OBJECTDIR}/map.o map.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/node.o: node.c  .generated_files/flags/default/c3747d3f6af493a0675c51c151589096133a5f7f .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/node.o.d 
	@${RM} ${OBJECTDIR}/node.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/node.o.d" -o ${OBJECTDIR}/node.o node.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/aux.o: aux.c  .generated_files/flags/default/92b109c061f926d89a4c608bd6c665b386d4db7a .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/aux.o.d 
	@${RM} ${OBJECTDIR}/aux.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/aux.o.d" -o ${OBJECTDIR}/aux.o aux.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/bot.o: bot.c  .generated_files/flags/default/3702dd968e66b79e18c4f4db366e19c6f314cce1 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/bot.o.d 
	@${RM} ${OBJECTDIR}/bot.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/bot.o.d" -o ${OBJECTDIR}/bot.o bot.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/imu.o: imu.c  .generated_files/flags/default/3a56a3d296f8c48d368a2f9d592617f775a161e0 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/imu.o.d 
	@${RM} ${OBJECTDIR}/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/imu.o.d" -o ${OBJECTDIR}/imu.o imu.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/master.o: master.c  .generated_files/flags/default/782e8bc363283490b0e9a153d6ac739917dd5759 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/master.o.d 
	@${RM} ${OBJECTDIR}/master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/master.o.d" -o ${OBJECTDIR}/master.o master.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/oled.o: oled.c  .generated_files/flags/default/787f90e16887481411e04020c6b0a690bd19ce37 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/oled.o.d 
	@${RM} ${OBJECTDIR}/oled.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/oled.o.d" -o ${OBJECTDIR}/oled.o oled.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/periph.o: periph.c  .generated_files/flags/default/e861b133c36c6582e2b4910aa0c50136b560aed4 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/periph.o.d 
	@${RM} ${OBJECTDIR}/periph.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/periph.o.d" -o ${OBJECTDIR}/periph.o periph.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/map.o: map.c  .generated_files/flags/default/5df199a1fbf941a4371596818a792b86f7858e18 .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/map.o.d 
	@${RM} ${OBJECTDIR}/map.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/map.o.d" -o ${OBJECTDIR}/map.o map.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/node.o: node.c  .generated_files/flags/default/6f314a02f16a1c51f938780444a3d788b73b17eb .generated_files/flags/default/dd3623c89e4677f90469d78ff507bf2f23022ec
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/node.o.d 
	@${RM} ${OBJECTDIR}/node.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -fno-common -MP -MMD -MF "${OBJECTDIR}/node.o.d" -o ${OBJECTDIR}/node.o node.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC00490:0x1FC00BEF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=_min_heap_size=32000,--defsym=_min_stack_size=100,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=32000,--defsym=_min_stack_size=100,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Master.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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
