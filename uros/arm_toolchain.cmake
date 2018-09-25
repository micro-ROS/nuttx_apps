include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Linux)
#set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_SYSROOT /root/)

#set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(BUILD_SHARED_LIBS OFF)
set(BUILD_TESTING OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)

link_libraries(c board drivers configs binfmt sched apps fs arch mm)
set(CMAKE_C_FLAGS_INIT "-std=c99 -nostdlib -Wl,--start-group -lc -lboard -ldrivers -lconfigs -lbinfmt -lsched -lapps -lfs -larch -lmm -Wl,--end-group" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++11 -nostdlib -Wl,--start-group -lc -lboard -ldrivers -lconfigs -lbinfmt -lsched -lapps -lfs -larch -lmm -lxx -Wl,--end-group" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-static")
#set(CMAKE_EXE_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libuClibc++")

include_directories(SYSTEM 
    /root/nuttx/include
    /root/nuttx/include/cxx
    /root/nuttx/include/uClibc++
#    /root/nuttx/include/cxx
#    /usr/uClibc++/include
#    /usr/lib/arm-none-eabi/include
#    /usr/include/x86_64-linux-gnu
#    /usr/include/newlib
#    /usr/include
    )

link_directories(
    /root/nuttx/staging
    /root/nuttx/configs/olimex-stm32-e407/src
    )


set(__BIG_ENDIAN__ 0)
