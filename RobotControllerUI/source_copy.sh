#!/bin/bash

echo 'file copy'
cp DataControl/*.h ../Deploy/DARControllerUI/DataControl/
cp FileIO/*.h ../Deploy/DARControllerUI/FileIO/
cp Input/*.h ../Deploy/DARControllerUI/Input/
cp Logger/*.h ../Deploy/DARControllerUI/Logger/
cp MainWindow/*.h ../Deploy/DARControllerUI/MainWindow/
cp MainWindow/*.ui ../Deploy/DARControllerUI/MainWindow/
cp OperateUI/*.h ../Deploy/DARControllerUI/OperateUI/
cp OperateUI/*.ui ../Deploy/DARControllerUI/OperateUI/
cp Settings/*.h ../Deploy/DARControllerUI/Settings/
cp TcpSocket/*.h ../Deploy/DARControllerUI/TcpSocket/
cp TorqueID/*.h ../Deploy/DARControllerUI/TorqueID/
cp TorqueID/*.ui ../Deploy/DARControllerUI/TorqueID/
cp -r lib ../Deploy/DARControllerUI

echo 'complete!!'
