# UAVGP2018

This project is based on DJI SDK 3.8.1.  
Our code can be found in `sample/linux/grab-task`.  

It's a simplified version from original source code.  
To protect personal privacy, some parameters are deleted.  
So it's cannot be directly used on an UAV.  

## Mechanism

It's quite simple: FSM(finite-state machine), to switch from one state to another.  

There is a GCS(ground contronl station) which continuously receives position infomation from the drone.  
And, of course, onboard computer runs a process(namely `UAVPG2018` program) keepping sending info to GCS.  
Before taking off, the staff should carry the drone to where the objects located and the GCS will record the position data.  
Then, GCS sends position data to onboard computer. During the grabbing task, the rotorcraft flies to these recorded places by GPS and RTK.  

The communication protocol (`userial/`) between GCS and onboard computer seems complex. It just reads and sends info by FSM, and you can also write by yourself.

## About the GCS

You can write it by yourself.
