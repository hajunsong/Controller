#############################################################
# Minimum required version                                  #
#############################################################
cmake_minimum_required(VERSION 3.16)

#############################################################
# Project name                                              #
#############################################################
project(RobotSim)

#############################################################
# Environment variables                                     #
#############################################################
# set(NRMK_CORE_INC_PATH /opt/neuromekaNRMKFoundation/core/include)
# set(NRMK_EIGEN_INC_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/eigen)
# set(NRMK_HELPER_INC_PATH /opt/neuromeka/NRMKFoundation/helper/include)
# set(NRMK_BIN_PATH /opt/neuromeka/NRMKFoundation/bin)
# set(NRMK_LIB_PATH /opt/neuromeka/NRMKFoundation/lib)
# set(NRMK_PATH = /opt/neuromeka/NRMKFoundation)
# set(NRMK_POCO_INC_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/include)
# set(NRMK_POCO_LIB_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib)
# set(NRMK_COLLISION_INC_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/collision/include)
# set(NRMK_COLLISION_LIB_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/collision/lib)
# set(NRMK_MODBUS_INC_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/modbus/include)
# set(NRMK_MODBUS_LIB_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/modbus/lib)
# set(NRMK_JSON_INC_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/json/include)
# set(NRMK_JSON_LIB_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/json/lib)
# set(NRMK_BOOST_INC_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/boost_1_64_0)
# set(NRMK_BOOST_LIB_PATH /opt/neuromeka/NRMKFoundation/core/3rdparty/boost_1_64_0/lib)
# set(NRMK_XENO_PATH /opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai)
# set(NRMK_ECAT_PATH /opt/neuromeka/NRMKPlatformPC2/bin/i686/ethercat)
# set(NRMK_TOOLCHAIN_PC2 /opt/neuromeka/NRMKPlatformPC2/bin/i686-unknown-linux-gnu/bin)

#############################################################
# Toolchains                                                #
#############################################################
# set(CMAKE_C_COMPILER ${NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER ${NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-g++)

set(CMAKE_C_COMPILER "/opt/poky/1.8/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-gcc")
#set(CMAKE_CXX_COMPILER "/opt/poky/1.8/sysroots/i686-pokysdk-linux/usr/bin/arm-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++")

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
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # ${NRMK_XENO_PATH}/include
    # ${NRMK_POCO_INC_PATH}
    # ${NRMK_POCO_LIB_PATH}/i686
    # ${NRMK_HELPER_INC_PATH}
    # ${NRMK_HELPER_INC_PATH}/hw
    /opt/xenomai-2.6.4/include
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
    src/fileio.cpp
    src/numerical.cpp
    src/robotarm.cpp
)

#############################################################
# Library                                                   #
#############################################################
set(LIBS
    pthread
    xenomai
    native
    rtdm
    # NRMKHelperi686
    # PocoNet
    # PocoFoundation
    # NRMKhw_tp
)

link_directories(
    /opt/xenomai-2.6.4/lib
    # ${NRMK_XENO_PATH}/lib
    # ${NRMK_ECAT_PATH}/lib
    # ${NRMK_LIB_PATH}
    # ${NRMK_POCO_LIB_PATH}/i686
)

#############################################################
# Executable                                                #
#############################################################
set(TARGET_EXE RobotSim)
add_executable(${TARGET_EXE} src/main.cpp ${SRCS})
set_target_properties(${TARGET_EXE} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${TARGET_EXE} ${LIBS})
