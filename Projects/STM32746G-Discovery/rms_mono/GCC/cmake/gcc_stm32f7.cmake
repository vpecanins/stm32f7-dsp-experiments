SET(CORE_FLAGS "-mthumb -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -mabi=aapcs")
SET(OPTIM_FLAGS "-fno-builtin -ffunction-sections -fdata-sections -fomit-frame-pointer -fno-unroll-loops -ffast-math -ftree-vectorize")

SET(CMAKE_C_FLAGS "${CORE_FLAGS} ${OPTIM_FLAGS} -Wall -std=gnu99" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "${CORE_FLAGS} ${OPTIM_FLAGS} -Wall -std=c++11" CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "${CORE_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections ${CORE_FLAGS}" CACHE INTERNAL "executable linker flags")
SET(CMAKE_MODULE_LINKER_FLAGS "${CORE_FLAGS}" CACHE INTERNAL "module linker flags")
SET(CMAKE_SHARED_LINKER_FLAGS "${CORE_FLAGS}" CACHE INTERNAL "shared linker flags")

# Preprocessor defines specific for stm32 device 
# The accepted values are copied from file CMSIS/Device/ST/STM32F7xx/stm32f7xx.h
# In theory, this is only needed if we are using STM32 Cube (99% of the time)
# But it cannot be put in FindSTM32CUBE.cmake bc that script is called with FIND_PACKAGE....

SET(STM32_ACCEPTED_DEF STM32F722xx STM32F723xx STM32F732xx 
						   STM32F733xx STM32F756xx STM32F746xx 
						   STM32F745xx STM32F765xx STM32F767xx 
						   STM32F769xx STM32F777xx STM32F779xx)

FOREACH(i ${STM32_ACCEPTED_DEF})
	STRING(REPLACE "xx" ".*" T1 ${i})
	SET(T2 "^${T1}$")
	IF(${CHIP} MATCHES ${T2})
		SET(STM32_CHIP_DEFINE ${i})
	ENDIF()
ENDFOREACH()

IF(STM32_CHIP_DEFINE)
	MESSAGE(STATUS "Chip define: ${STM32_CHIP_DEFINE}")
ELSE()
	MESSAGE(WARNING "Cannot find any matching CHIP_DEFINE (Needed for STM32 HAL libraries)")
ENDIF()

ADD_DEFINITIONS("-D${STM32_CHIP_DEFINE} -DARM_MATH_CM7")
