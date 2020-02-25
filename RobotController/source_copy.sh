#!/bin/bash

echo 'copy CotrolMain'
mkdir -p ../RobotControllerLib/ControlMain
cp ControlMain/controlmain.cpp ../RobotControllerLib/ControlMain
mkdir -p ../Deploy/FARController/ControlMain
cp ControlMain/controlmain.h ../Deploy/FARController/ControlMain

echo 'copy CustomFunc'
mkdir -p ../RobotControllerLib/CustomFunc
cp CustomFunc/controlmain_custom.cpp ../RobotControllerLib/CustomFunc
cp CustomFunc/controlmain_custom.h ../RobotControllerLib/CustomFunc
cp CustomFunc/tcpserver_custom.cpp ../RobotControllerLib/CustomFunc
cp CustomFunc/tcpserver_custom.h ../RobotControllerLib/CustomFunc
mkdir -p ../Deploy/FARController/CustomFunc
cp CustomFunc/controlmain_custom.cpp ../Deploy/FARController/CustomFunc
cp CustomFunc/controlmain_custom.h ../Deploy/FARController/CustomFunc
cp CustomFunc/tcpserver_custom.cpp ../Deploy/FARController/CustomFunc
cp CustomFunc/tcpserver_custom.h ../Deploy/FARController/CustomFunc

echo 'copy DataControl'
mkdir -p ../RobotControllerLib/DataControl
cp DataControl/datacontrol.cpp ../RobotControllerLib/DataControl
mkdir -p ../Deploy/FARController/DataControl
cp DataControl/datacontrol.h ../Deploy/FARController/DataControl

echo 'copy Dynamixel'
mkdir -p ../RobotControllerLib/Dynamixel
cp Dynamixel/dynamixel.cpp ../RobotControllerLib/Dynamixel
cp Dynamixel/libdxl_x86_cpp.so ../RobotControllerLib/Dynamixel
cp -r Dynamixel/dynamixel_sdk ../RobotControllerLib/Dynamixel
mkdir -p ../Deploy/FARController/Dynamixel
cp Dynamixel/dynamixel.h ../Deploy/FARController/Dynamixel
cp Dynamixel/libdxl_x86_cpp.so ../Deploy/FARController/Dynamixel
cp -r Dynamixel/dynamixel_sdk ../Deploy/FARController/Dynamixel

echo 'copy FileIO'
mkdir -p ../RobotControllerLib/FileIO
cp FileIO/fileio.cpp ../RobotControllerLib/FileIO
mkdir -p ../Deploy/FARController/FileIO
cp FileIO/fileio.h ../Deploy/FARController/FileIO

echo 'copy Input'
mkdir -p ../RobotControllerLib/Input
cp Input/keyinput.cpp ../RobotControllerLib/Input
mkdir -p ../Deploy/FARController/Input
cp Input/keyinput.h ../Deploy/FARController/Input

echo 'copy RobotArm'
mkdir -p ../RobotControllerLib/RobotArm
cp RobotArm/numerical.cpp ../RobotControllerLib/RobotArm
cp RobotArm/robotarm.cpp ../RobotControllerLib/RobotArm
mkdir -p ../Deploy/FARController/RobotArm
cp RobotArm/numerical.h ../Deploy/FARController/RobotArm
cp RobotArm/robotarm.h ../Deploy/FARController/RobotArm

echo 'copy Settings'
mkdir -p ../RobotControllerLib/Settings
cp Settings/customsettings.cpp ../RobotControllerLib/Settings
mkdir -p ../Deploy/FARController/Settings
cp Settings/customsettings.h ../Deploy/FARController/Settings

echo 'copy TcpServer'
mkdir -p ../RobotControllerLib/TcpServer
cp TcpServer/tcpserver.cpp ../RobotControllerLib/TcpServer
mkdir -p ../Deploy/FARController/TcpServer
cp TcpServer/tcpserver.h ../Deploy/FARController/TcpServer
