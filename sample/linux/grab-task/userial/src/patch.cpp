#include "patch.h"
#include <dji_vehicle.hpp>

//v3.2.2->v3.8.1的补丁文件

BroadcastData getBroadcastData(DJI::OSDK::Vehicle *vehicle)
{
    BroadcastData data;
    data.pos=vehicle->broadcast->getGlobalPosition();
    data.q=vehicle->broadcast->getQuaternion();
    data.v=vehicle->broadcast->getVelocity();
    data.w=vehicle->broadcast->getAngularRate();
    data.rtk=vehicle->broadcast->getRTKInfo();
    data.battery=vehicle->broadcast->getBatteryInfo();

    return data;
}