/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "flight_control_sample.hpp"

#include "../userial/inc/gcs_thread.h"
#include "../userial/inc/protocol.h"
#include<pthread.h>

#include "patch.h"
#include <sys/time.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


extern void GrabTask(Vehicle* vehicle, struct timeval tv, double yawFixed, void* data=NULL);

/*! main
 *
 */
int main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  if(startGlobalBroadcast(vehicle))
  {
    // Wait for Broadcast ready
    sleep(2);

/**************************************************************
 *  设置地面站，打点等
 *************************************************************/
    pthread_attr_t attr;
    pthread_t gcsthread;
    
    if(gcs_interface_init()==-1)  {
      // for camera thread
      DSTATUS("gcs init finished");
      if(pthread_attr_destroy(&attr) != 0)
      {
        DERROR("pthread_attr_destroy error");
      }
    }

    while(1){
      BroadcastData data;
      data = getBroadcastData(vehicle);
      setUAVstatus((uint8_t *)&(data),sizeof(BroadcastData));
      usleep(200*1000);
      if(getControlStatus()) break;
    }

    if((pthread_create(&gcsthread,NULL,GCSThreading,NULL))!= 0)
    {
      printf("gcs create error!!\n");
      return -1;
    }

/**************************************************************
 *  记录初始角度，地面高度等
 *************************************************************/
    // 飞行过程中的航向角和飞机起飞前放置的角度一致
    BroadcastData bcData = getBroadcastData(vehicle);
    Telemetry::Vector3f atti = toEulerAngle((static_cast<void*>(&(bcData.q))));
    //printf("origin height  = %10.3f \n",rtk_begin.Hmsl);
    
    setHmsOrigin(bcData.rtk.pos.data.HFSL);
    setHmslInfo(getHmsOrigin());
    DSTATUS("HmslOrigin=%f",getHmsOrigin());
    
    double yawFixed = atti.z*180/3.1415926535898;
    if(yawFixed>180)
      yawFixed-=360;
    DSTATUS("yawFixed=%f\n",yawFixed);
    
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
/**************************************************************
 *  开始任务
 *************************************************************/

    if(monitoredTakeoff(vehicle))
    {
      GrabTask(vehicle, tv, yawFixed);
    }
    monitoredLanding(vehicle);
  }
  else
  {
    DERROR("Cannot open boradcast.\n");
    return -1;
  }

  return 0;
}
