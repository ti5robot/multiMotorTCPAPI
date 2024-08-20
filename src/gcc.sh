#!/bin/bash
export CPLUS_INCLUDE_PATH=/mnt/hgfs/multi_motor_new/include:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/mnt/hgfs/multi_motor_new/include/can
g++ main.cpp  Ti5BASIC.cpp Ti5LOGIC.cpp  mathfunc.cpp  tool.cpp socket_service.cpp -L../include/can -lmylibscan -lcontrolcan -o multi_motor_new
