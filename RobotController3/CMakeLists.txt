#############################################################
# Minimum required version                                  #
#############################################################
cmake_minimum_required(VERSION 3.16)

#############################################################
# Environment variables                                     #
#############################################################
set(NRMK_BIN_PATH "/opt/neuromeka/NRMKFoundation/bin")
set(NRMK_BOOST_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/boost_1_64_0")
set(NRMK_BOOST_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/boost_1_64_0/lib")
set(NRMK_BOOST_X64_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/boost/x86_64/include")
set(NRMK_BOOST_X64_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/boost/x86_64/lib")
set(NRMK_COLLISION_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/collision/include")
set(NRMK_COLLISION_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/collision/lib")
set(NRMK_COLLISION_X64_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/collision/x86_64/include")
set(NRMK_COLLISION_X64_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/collision/x86_64/lib")
set(NRMK_CORE_INC_PATH "/opt/neuromeka/NRMKFoundation/core/include")
set(NRMK_ECAT_PATH "/opt/neuromeka/NRMKPlatformPC2/bin/i686/ethercat")
set(NRMK_EIGEN_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/eigen")
set(NRMK_EIGEN_X64_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/eigen/x86_64")
set(NRMK_HELPER_INC_PATH "/opt/neuromeka/NRMKFoundation/helper/include")
set(NRMK_JSON_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/json/include")
set(NRMK_JSON_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/json/lib")
set(NRMK_JSON_X64_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/json/x86_64/include")
set(NRMK_JSON_X64_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/json/x86_64/lib")
set(NRMK_LIB_PATH "/opt/neuromeka/NRMKFoundation/lib")
set(NRMK_LIB_X64_PATH "/opt/neuromeka/NRMKFoundation/lib64")
set(NRMK_MODBUS_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/modbus/include")
set(NRMK_MODBUS_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/modbus/lib")
set(NRMK_MODBUS_X64_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/modbus/x86_64/include/modbus")
set(NRMK_MODBUS_X64_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/modbus/x86_64/lib")
set(NRMK_POCO_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/include")
set(NRMK_POCO_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/lib")
set(NRMK_POCO_X64_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/x86_64/include")
set(NRMK_POCO_X64_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/Poco/x86_64/lib")
set(NRMK_TOOLCHAIN_PC2 "/opt/neuromeka/NRMKPlatformPC2/bin/i686-unknown-linux-gnu/bin")
set(NRMK_VISION_X64_INC_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/vision/x86_64/include")
set(NRMK_VISON_X64_LIB_PATH "/opt/neuromeka/NRMKFoundation/core/3rdparty/vision/x86_64/lib")
set(NRMK_XENO_PATH "/opt/neuromeka/NRMKPlatformPC2/bin/i686/xenomai")

#############################################################
# Toolchains                                                #
#############################################################
set(CMAKE_C_COMPILER "${NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "${NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-g++")

#############################################################
# Project name                                              #
#############################################################
project(DARController)

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
    ${NRMK_XENO_PATH}/include
    ${NRMK_POCO_INC_PATH}
    ${NRMK_POCO_LIB_PATH}/i686
    ${NRMK_HELPER_INC_PATH}
    ${NRMK_HELPER_INC_PATH}/hw
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
    ${NRMK_XENO_PATH}/lib
    ${NRMK_ECAT_PATH}/lib
    ${NRMK_LIB_PATH}
    ${NRMK_POCO_LIB_PATH}/i686
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

#############################################################
# Executable                                                #
#############################################################
set(TARGET_EXE DARController)
add_executable(${TARGET_EXE} src/main.cpp ${SRCS})

#############################################################
# Deployment                                                #
#############################################################
#set(DEPLOY_LIB DARControllerLib)
#add_library(${DEPLOY_LIB} SHARED ${SRCS})

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${TARGET_EXE} ${LIBS})
#target_link_libraries(${DEPLOY_LIB} ${LIBS})

set_target_properties(${TARGET_EXE} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
#set_target_properties(${DEPLOY_LIB} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib")

