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
project(FARController)

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
	src/main.cpp
	src/controlmain_custom.cpp
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
	FARControllerLib
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
set(TARGET_EXE FARController2)
add_executable(${TARGET_EXE} ${SRCS})

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${TARGET_EXE} ${LIBS})

set_target_properties(${TARGET_EXE} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin")