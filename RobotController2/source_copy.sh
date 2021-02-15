#!/bin/bash

echo 'copy CotrolMain'
mkdir -p ../Deploy/FARController2/ControlMain
cp ControlMain/controlmain.h ../Deploy/FARController2/ControlMain

echo 'copy CustomFunc'
mkdir -p ../Deploy/FARController2/CustomFunc
cp CustomFunc/controlmain_custom.cpp ../Deploy/FARController2/CustomFunc
cp CustomFunc/controlmain_custom.h ../Deploy/FARController2/CustomFunc

echo 'copy DataControl'
mkdir -p ../Deploy/FARController2/DataControl
cp DataControl/datacontrol.h ../Deploy/FARController2/DataControl

echo 'copy Dynamixel'
mkdir -p ../Deploy/FARController2/Dynamixel
cp Dynamixel/dynamixel.h ../Deploy/FARController2/Dynamixel
cp Dynamixel/libdxl_x86_cpp.so ../Deploy/FARController2/Dynamixel
cp -r Dynamixel/dynamixel_sdk ../Deploy/FARController2/Dynamixel

echo 'copy FileIO'
mkdir -p ../Deploy/FARController2/FileIO
cp FileIO/fileio.h ../Deploy/FARController2/FileIO

echo 'copy RobotArm'
mkdir -p ../Deploy/FARController2/RobotArm
cp RobotArm/numerical.h ../Deploy/FARController2/RobotArm
cp RobotArm/robotarm.h ../Deploy/FARController2/RobotArm

echo 'copy TcpServer'
mkdir -p ../Deploy/FARController2/TcpServer
cp TcpServer/tcpserver.h ../Deploy/FARController2/TcpServer
