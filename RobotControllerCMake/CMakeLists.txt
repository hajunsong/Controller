cmake_minimum_required(VERSION 3.9)

project(RobotController LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_C_COMPILER "$ENV{NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-gcc")
#set(CMAKE_CXX_COMPILER "$ENV{NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-g++")

set(CMAKE_BUILD_TYPE Release)

set(INSTDIR "/opt/neuromka")

include_directories(
	${INSTDIR}/NRMKFoundation/core/include
	${INSTDIR}/NRMKFoundation/core/3rdparty/eigen
	${INSTDIR}/NRMKFoundation/helper/include
	${INSTDIR}/NRMKFoundation/bin
	${INSTDIR}/NRMKFoundation/lib
	${INSTDIR}/NRMKFoundation
	${INSTDIR}/NRMKFoundation/core/3rdparty/Poco/include
	${INSTDIR}/NRMKFoundation/core/3rdparty/Poco/lib
	${INSTDIR}/NRMKFoundation/core/3rdparty/collision/include
	${INSTDIR}/NRMKFoundation/core/3rdparty/collision/lib
	${INSTDIR}/NRMKFoundation/core/3rdparty/modbus/include
	${INSTDIR}/NRMKFoundation/core/3rdparty/modbus/lib
	${INSTDIR}/NRMKFoundation/core/3rdparty/json/include
	${INSTDIR}/NRMKFoundation/core/3rdparty/json/lib
	${INSTDIR}/NRMKFoundation/core/3rdparty/boost_1_64_0
	${INSTDIR}/NRMKFoundation/core/3rdparty/boost_1_64_0/lib
	${INSTDIR}/NRMKPlatformPC2/bin/i686/xenomai
	${INSTDIR}/NRMKPlatformPC2/bin/i686/ethercat
	${INSTDIR}/NRMKPlatformPC2/bin/i686-unknown-linux-gnu/bin
)

set(LIBS
	xenomai
	native
	rtdm
	NRMKHelperi686
	PocoNet
	PocoFoundation
	dxl_x86_cpp
)

set(SRCS
	ControlMain/controlmain.cpp
	CustomFunc/controlmain_custom.cpp
	CustomFunc/tcpserver_custom.cpp
	DataControl/datacontrol.cpp
	Dynamixel/dynamixel.cpp
	FileIO/fileio.cpp
	Input/keyinput.cpp
	RobotArm/numerical.cpp
	RobotArm/robotarm.cpp
	TcpServer/tcpserver.cpp
)

link_directories(
	"$ENV{NRMK_XENO_PATH}/lib"
	"$ENV{NRMK_ECAT_PATH}/lib"
	"$ENV{NRMK_LIB_PATH}"
	"$ENV{NRMK_POCO_LIB_PATH}/i686"
)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBS})

set(CMAKE_INSTALL_PREFIX "/home/user/Project/RobotControllerCMake")
install(TARGETS ${PROJECT_NAME} 
	    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
