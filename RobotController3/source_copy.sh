#!/bin/bash

echo 'make directory'
mkdir -p ../Deploy/FARController3/src
mkdir -p ../Deploy/FARController3/include
mkdir -p ../Deploy/FARController3/lib

echo 'file copy'
cp src/controlmain_custom.cpp ../Deploy/FARController3/src
cp src/main.cpp ../Deploy/FARController3/src
cp -r include ../Deploy/FARController3
cp -r lib ../Deploy/FARController3

echo 'complete!!'