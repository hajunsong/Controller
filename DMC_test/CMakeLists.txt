#############################################################
# Minimum required version                                  #
#############################################################
cmake_minimum_required(VERSION 3.16)

#############################################################
# Project name                                              #
#############################################################
project(DMC_test)

#############################################################
# Toolchains                                                #
#############################################################
set(CMAKE_C_COMPILER "/opt/poky/1.8/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-gcc")

#############################################################
# Build type                                                #
#############################################################
set(CMAKE_BUILD_TYPE Release)

#############################################################
# Includes                                                  #
#############################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    /opt/xenomai-2.6.4/include
)

# Release flags
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

# Debug flags
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -O0")

#############################################################
# Sources                                                   #
#############################################################
set(SRCS
    src/custom_slave_information.cpp
    src/daininterface.cpp
    src/ecatmaster.cpp
    src/ecatslave.cpp
    src/userinterface.cpp
)

#############################################################
# Library                                                   #
#############################################################
set(LIBS
    pthread
    xenomai
    native
    rtdm
)

link_directories(
    /opt/xenomai-2.6.4/lib
)

#############################################################
# Executable                                                #
#############################################################
set(TARGET_EXE DMC_test)
add_executable(${TARGET_EXE} src/main.cpp ${SRCS})
set_target_properties(${TARGET_EXE} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${TARGET_EXE} ${LIBS})
