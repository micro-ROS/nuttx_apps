include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_SYSROOT /root/)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(BUILD_SHARED_LIBS OFF)
set(BUILD_TESTING OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Makefile flags
set(CROSSDEV "" CACHE STRING "GCC compiler use in NuttX.")
set(ARCH_CPU_FLAGS "" CACHE STRING "Makefile arquitecture flags.")
set(ARCH_OPT_FLAGS "" CACHE STRING "Makefile optimization flags.")

# Compiler tools
foreach(tool gcc ld ar)
	string(TOUPPER ${tool} TOOL)
    find_program(${TOOL} ${CROSSDEV}${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

CMAKE_FORCE_C_COMPILER(${CROSSDEV}gcc GNU)
CMAKE_FORCE_CXX_COMPILER(${CROSSDEV}g++ GNU)

set(CMAKE_C_FLAGS_INIT "-std=c99 ${ARCH_CPU_FLAGS} ${ARCH_OPT_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++11 ${ARCH_CPU_FLAGS} ${ARCH_OPT_FLAGS} " CACHE STRING "" FORCE)


include_directories(SYSTEM 
    /root/nuttx/include
    /root/nuttx/include/cxx
    /root/nuttx/include/uClibc++
    )

link_directories(
    /root/nuttx/staging
    /root/nuttx/configs/olimex-stm32-e407/src
    )
    
set(__BIG_ENDIAN__ 0)