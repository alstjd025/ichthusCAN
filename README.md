ichthusCAN
==============
# About

This repository contains code for CAN(Control Area Network) Communication in C/C++ & Decode 

# How To Use

How To Test in VCAN0 (Virtual Can)    
     
'sudo apt install net-tools'    
'sudo apt install can-utils'    
'sudo modprobe vcan'    
'sudo ip link add dev vcan0 type vcan'    
'sudo ip link set up vcan0'    
'candump vcan0'    
    
After this, just open another terminal and command    
'cansend vcan0 386#11.11.11.11.11.11.11.11'    
And you can see the message you've sent shows right up in candump terminal.    
    
In libcan directory where libcan.so and other so files exist, command 'make'       
CD into test directory and 'make'    
Now you can run test programm with './test hyundai_kia_generic.dbc'     

# libcan
Link: [libcan][libcanlink]

[libcanlink]: https://github.com/matthiasbock/libcan

# dbcppp
Link: [dbcppp][dbcppplink]

[dbcppplink]: https://github.com/xR3b0rn/dbcppp

## Features
It aims to support severer CAN devices:

Linux:
  * all SocketCAN and SLCAN devices, specifically:
  * CANtact (SLCAN)
  * Peak-Systeme PCAN-USB (SocketCAN)

Decoding CAN Raw data by using DBCPPP library

Planned features:
* Transmitting Can Frames to connected CAN-bus

# License
Author : 
  * Minsung Kim <alstjd025@naver.com>   
  * Heesoo Kim <mlksvender@gmail.com>
  * Jonghyeok Yoo <youjong12@naver.com>   

      
Mobility Intelligence & Computing Systems Laboratory, Soongsil University.    
Use only in ICHTHUS2022 Project for 2022 University Student Autonomous Driving Competition, MOTIE, Korea government.     



