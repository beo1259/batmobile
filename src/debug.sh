#!/bin/sh
clang++ --debug main.cpp -o main -I/usr/include -L/usr/lib -lpigpio -lpigpiod_if2 -levdev
sudo setcap cap_sys_rawio,cap_dac_override+ep ./main
