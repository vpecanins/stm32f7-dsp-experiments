SET(CORE_FLAGS "-mthumb -mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -mabi=aapcs")
SET(OPTIM_FLAGS "-fno-builtin -ffunction-sections -fdata-sections -fomit-frame-pointer -fno-unroll-loops -ffast-math -ftree-vectorize")

SET(CMAKE_C_FLAGS "${CORE_FLAGS} ${OPTIM_FLAGS} -Wall -std=gnu99" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "${CORE_FLAGS} ${OPTIM_FLAGS} -Wall -std=c++11" CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "${CORE_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections ${CORE_FLAGS}" CACHE INTERNAL "executable linker flags")
SET(CMAKE_MODULE_LINKER_FLAGS "${CORE_FLAGS}" CACHE INTERNAL "module linker flags")
SET(CMAKE_SHARED_LINKER_FLAGS "${CORE_FLAGS}" CACHE INTERNAL "shared linker flags")


