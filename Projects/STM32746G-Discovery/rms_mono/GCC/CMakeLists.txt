# CMake script for building projects on STM32 / Cortex-M Microcontrollers
# Copyright 2017 Victor Pecanins
#
# In Windows:
# - Use GIT Bash MSYS console
# - Use cmake for windows version (add it to the GIT Bash $PATH, in ~/.bashrc)
# - Install msys-make in the GIT Bash with mingw-get
# - gcc-arm-none-eabi for windows
# - Generate Makefiles calling: cmake . -G"MSYS Makefiles"
#
# In Linux:
# <Not yet tested>

CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

# Uncomment this line to print the full command line in Makefile
#SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set the CHIP variable to the part number of your uC
SET(CHIP "STM32F746NG" CACHE STRING "Chip part number")

LIST(APPEND CMAKE_MODULE_PATH "./cmake/")
INCLUDE(gcc_arm_cortex)

FIND_PACKAGE(STM32CUBE)

GET_FILENAME_COMPONENT(PROJECT_ROOT "../" ABSOLUTE)
GET_FILENAME_COMPONENT(PROJ_NAME ${PROJECT_ROOT} NAME)

PROJECT(${PROJ_NAME})
ENABLE_LANGUAGE(ASM)

INCLUDE_DIRECTORIES(
	${PROJECT_ROOT}/Inc
    ${STM32CUBE_INC_DIRS}
)

FILE(GLOB C_SOURCE_FILES "${PROJECT_ROOT}/Src/*.c")
SET(PROJECT_SOURCES
    ${C_SOURCE_FILES}
)

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${PROJECT_SOURCES}) 

TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf -Wl,--start-group ${STM32CUBE_LIBS} -Wl,--end-group)

ARM_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME}.elf)

install(CODE "MESSAGE(\"Sample install message.\")")
