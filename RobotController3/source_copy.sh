#!/bin/bash

echo 'make directory'
mkdir -p ../Deploy/DARController/src
mkdir -p ../Deploy/DARController/include
mkdir -p ../Deploy/DARController/lib

echo 'file copy'
cp src/controlmain_custom.cpp ../Deploy/DARController/src
cp src/main.cpp ../Deploy/DARController/src
cp -r include ../Deploy/DARController
cp -r lib ../Deploy/DARController

echo 'complete!!'