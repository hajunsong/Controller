#############################################################
# Minimum required version                                  #
#############################################################
cmake_minimum_required(VERSION 2.8)

#############################################################
# Toolchains                                                #
#############################################################
set(CMAKE_C_COMPILER "$ENV{NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "$ENV{NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-g++")

#############################################################
# Project name                                              #
#############################################################
project(RobotController2)

#############################################################
# C, CPP Standard                                           #
#############################################################
# C standard
set(CMAKE_C_STANDARD 11)

# CPP standard
set(CMAKE_CXX_STANDARD 11)

#############################################################
# Build type                                                #
#############################################################
set(CMAKE_BUILD_TYPE 
	Release
	# Debug
)

#############################################################
# Includes                                                  #
#############################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(
	$ENV{NRMK_XENO_PATH}/include
	$ENV{NRMK_POCO_INC_PATH}
	$ENV{NRMK_POCO_LIB_PATH}/i686
	$ENV{NRMK_HELPER_INC_PATH}
	$ENV{NRMK_HELPER_INC_PATH}/hw
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0")
		
#############################################################
# Sources                                                   #
#############################################################
set(SRCS
	src/controlmain.cpp
	src/controlmain_custom.cpp
	src/datacontrol.cpp
	src/dynamixel.cpp
	src/fileio.cpp
	src/numerical.cpp
	src/robotarm.cpp
	src/tcpserver.cpp
)

#############################################################
# Library                                                   #
#############################################################
set(LIBS
	xenomai
	native
	rtdm
	NRMKHelperi686
	PocoNet
	PocoFoundation
	pthread
	NRMKhw_tp
	dxl_x86_cpp
)

link_directories(
	$ENV{NRMK_XENO_PATH}/lib
	$ENV{NRMK_ECAT_PATH}/lib
	$ENV{NRMK_LIB_PATH}
	$ENV{NRMK_POCO_LIB_PATH}/i686
	${CMAKE_CURRENT_SOURCE_DIR}/lib
)

#############################################################
# Executable                                                #
#############################################################
set(TARGET_EXE RobotController2)
add_executable(${TARGET_EXE} src/main.cpp ${SRCS})

#############################################################
# Deployment                                                #
#############################################################
set(DEPLOY_LIB FARControllerLib)
add_library(${DEPLOY_LIB} SHARED ${SRCS})

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${TARGET_EXE} ${LIBS})
target_link_libraries(${DEPLOY_LIB} ${LIBS})

set_target_properties(${TARGET_EXE} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin")
# set_target_properties(${DEPLOY_LIB} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/lib")
set_target_properties(${DEPLOY_LIB} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../Deploy/FARController3/lib")