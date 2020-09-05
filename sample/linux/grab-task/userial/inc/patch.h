#ifndef __PATCH_H__
#define __PATCH_H__

#include "dji_vehicle.hpp"
#include "dji_telemetry.hpp"

typedef struct
{
    DJI::OSDK::Telemetry::GlobalPosition pos;
    DJI::OSDK::Telemetry::Quaternion q;
    DJI::OSDK::Telemetry::Vector3f w;
    DJI::OSDK::Telemetry::Vector3f v;
    DJI::OSDK::Telemetry::RTK rtk;
    DJI::OSDK::Telemetry::Battery battery;
} BroadcastData;

BroadcastData getBroadcastData(DJI::OSDK::Vehicle *vehicle);

#endif