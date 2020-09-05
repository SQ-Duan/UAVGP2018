#include<iomanip>
#include "../userial/inc/protocol.h"
#include <string>
#include <fstream>
#include <pthread.h>
#include <fstream>
#include <sys/time.h>
#include <myutils.h>
#include <GlobalVars.h>
#include <LinuxGpio.h>
#include <dji_vehicle.hpp>
#include <dji_control.hpp>
#include "dji_log.hpp"
#include <cmath>
using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

//使用NED坐标系
//RTK单位<deg deg m>
//GlobalPositiond单位<rad rad m>

// 默认此文件全局使用弧度

//******************************************************
// 宏定义
//#define __SIMULATION__

#define C_EARTH (double) 6378137.0
#define pi 3.1415926535898 
#define DEG2RAD 0.01745329252

//******************************************************
// 定义类型

// to save posi info
typedef struct Vector3lf
{
  float64_t x;
  float64_t y;
  float64_t z;
} Vector3lf;

// 飞行状态
typedef enum {
  //起飞
  FLIGHT_TAKEOFF=0,
  
  //到达抓取点
  FLIGHT_ARRIVAL_STOCK_POINT,
  //识别出抓取物
  FLIGHT_RECOGNTION_SUCCESS,
  //抓取成功
  FLIGHT_GRABBING_SUCCESS,
  //到达投放点
  FLIGHT_ARRIVAL_DROP_POINT,
  //投放成功
  FLIGHT_DROPPING_SUCCESS,
  //开始新的一次抓取任务
  FLIGHT_NEW_GRUBBING,
  //任务完成：达到指定投放数目或达到指定时间
  FLIGHT_TASK_DONE,
  //电量不足
  FLIGHT_LOW_BATTERY,
  
  //到达降落点
  FLIGHT_ARRIVAL_LANDING_POINT,
} FlightState;

//******************************************************
// 函数声明

// 获取打点坐标
void GetRtkFlightMap(Vehicle* vehicle, std::vector<Vector3lf>& _rtkFlightMap);
// 计算当前位置到目标点距离
void GetRtkOffset(Vector3lf &deltaNed, Vector3lf &target, Vector3lf &origin);
// 输出当前位置信息
void printPos(Vehicle* vehicle);
// 控制飞机飞往坐标点
bool FlightControl(Vehicle* vehicle, float32_t yawDesired, Vector3lf &target, int timeoutInMs=15000, float posThresholdInM=0.05);
// 抓取过程控制
bool GrabControl(Vehicle* vehicle, float32_t yawDesired, Vector3lf &target, int timeoutInMs=15000, float posThresholdInM=0.05);
// 控制飞机垂直飞行，cmd用于控制垂直起降时是否放绳，cmd=true时，在垂直飞行的同时放绳
bool VerticalControl(Vehicle* vehicle, float32_t yawDesired, float32_t zVelocity, Vector3lf &target, bool cmd, int timeoutInMs=10000, float posThresholdInM=0.05);
//******************************************************
// 变量声明

// 飞行时间阈值，大于阈值返航
const double TIME_THRESHOLD=CONTROLARGS.timeThreshold;
// 搬运次数阈值，达到阈值返航
const int NUMDONE_THRESHOLD=CONTROLARGS.aimedDoneNumber;
// 抓取点打点个数
const int GRABPOINT_NUMBER=CONTROLARGS.nGrabPoints;
// 总共打点个数
const int POINT_NUMBER=GRABPOINT_NUMBER+3;
// 失败后最多尝试次数
const int TRY_TIMES=CONTROLARGS.tryTimes;
// 低于电池电量阈值将返航
const int BATTERY_THRESHOLD=CONTROLARGS.batteryThreshold;
// 抓取时飞机的悬停高度
const float GRAB_HEIGHT=CONTROLARGS.grabHeight;

ofstream flight_log("flight_log.txt", ios_base::out);

static bool first_drop = true;
//******************************************************
// 函数定义

// 获取程序运行时间
double time_offset(struct timeval begin)
{
	double cal_time;
	long begin_s,end_s;
	long begin_us,end_us;
	begin_us=begin.tv_usec;
	begin_s=begin.tv_sec;
	struct timeval end;
	gettimeofday(&end, NULL);
	end_us=end.tv_usec;
	end_s=end.tv_sec;
  cal_time=end_s-begin_s+(double)(end_us-begin_us)/1000000.0;
	return cal_time;
}

//获取当前位置信息 NED
//返回纬度(rad)、经度(rad)、高度(m)
inline Vector3lf GetCurPosition(Vehicle* vehicle)
{
  Vector3lf NED;
#ifdef __SIMULATION__
  //GPS原始数据为弧度值
  GlobalPosition tmp2=vehicle->broadcast->getGlobalPosition();
  NED.x=tmp2.latitude;
  NED.y=tmp2.longitude;
  NED.z=tmp2.height;
#else
  //RTK原始数据为角度值
  RTK tmp2=vehicle->broadcast->getRTKInfo();
  NED.x=tmp2.pos.data.latitude*DEG2RAD;
  NED.y=tmp2.pos.data.longitude*DEG2RAD;
  NED.z=tmp2.pos.data.HFSL;
#endif
  return NED;
}

void GrabTask(Vehicle* vehicle, struct timeval tv, double yawFixed, void* data)
{ 
  //ThreadData* const pCamThreadData=(ThreadData*)(data);
  DSTATUS("Enter Task.");
  
  double curTime=0;
  int curGrabPoint=-1;
  int numDone=0;
  int failedTimes=0;
  uint32_t batteryData;
  
  FlightState flightState=FLIGHT_TAKEOFF;
  int moveState=0;
  
  Vector3lf rtkOffset;
  
  Vector3lf curRtkData;

  //关闭抓取装置，并检测串口是否正常
  if(grabserial.write_cmd('1')==1)
	  for(int i=0;i<101;++i)
	  {
	    if(grabserial.read_ack()=='1')
	      break;
	    if(i==100)
	    {
	      cout << "Error: Cannot read ack." << endl;
	      return;
	    }
	    usleep(50000);
	  }
	else
	{
		cout << "Error: Cannot Send cmd." << endl;
		return;
	}
  
  std::vector<Vector3lf> rtkFlightMap;
  GetRtkFlightMap(vehicle, rtkFlightMap);
  size_t mapSize=rtkFlightMap.size();
  if(mapSize<POINT_NUMBER)
    return;
  else
    flight_log<<"MapSize:"<<mapSize<<endl;
  
  // 当一个抓取点被判定为不可抓取(抓取风险较大)时，在 validatedMap 设置标志位
  // 通过视觉判定风险，该功能目前已删除
  std::vector<bool> validatedMap(GRABPOINT_NUMBER,true);
  int invalidatedGrabPointNum=0;
  
  while(1)
  {
    //TODO: 飞行状态检测 手控/自动
    if(flightState==FLIGHT_ARRIVAL_LANDING_POINT)
      break;
    else
    {

      switch(flightState)
      {
/********************************************************************************
* 任务完成或电量不足时，返回着陆点。
*********************************************************************************/
      case FLIGHT_TASK_DONE:
      case FLIGHT_LOW_BATTERY:
        
        flight_log<<"Begin to return home now."<<endl;
        moveState=FlightControl(vehicle, yawFixed, rtkFlightMap[2], 15000, 0.2);
        if(moveState)
        {
          flightState=FLIGHT_ARRIVAL_LANDING_POINT;
          flight_log<<"Arrived at landing point."<<endl;
        }
        else
        {
          //TODO:对于超时的处理
        }
        break;
        
        
/********************************************************************************
* 起飞后，开始飞往指定起始点rtkFight
*********************************************************************************/
      case FLIGHT_TAKEOFF:
        
        flight_log<<"Fly to start point."<<endl;
        moveState=FlightControl(vehicle, yawFixed, rtkFlightMap[0], 10000, 0.2);
        //sleep(1);
        flightState=FLIGHT_NEW_GRUBBING;
        flight_log<<"Arrived at start point."<<endl;
        break;


/********************************************************************************
* 起飞或完成一次投放之后，开始新一轮的抓取
* 抓取失败次数清零
* 判断并选择可用的抓取点
*********************************************************************************/	  
      case FLIGHT_NEW_GRUBBING:
        
        failedTimes=0;
        // 遍历 validatedMap 寻找到一个可用抓取点就退出循环
        // 每轮任务之后 curGrabPoint+1, 保证先把各堆箱子的上层先抓完，避免只抓某一堆箱子
        invalidatedGrabPointNum=0;
        for(int i=0;i<GRABPOINT_NUMBER;++i)
        {
          ++curGrabPoint;
          if(curGrabPoint==GRABPOINT_NUMBER)
            curGrabPoint=0;
          if(validatedMap[curGrabPoint])
            break;
          else
            ++invalidatedGrabPointNum;
        }

        if(invalidatedGrabPointNum>=GRABPOINT_NUMBER)
        {
          cout << "No validated GrabPoint." <<endl;
          flight_log << "No validated GrabPoint." <<endl;
          flightState=FLIGHT_TASK_DONE;
        }
        else
        {
          flight_log<<"New grabbing starts, flying to stock point"<<endl;
          moveState=FlightControl(vehicle, yawFixed, rtkFlightMap[curGrabPoint+3], 15000, 0.2);
          //sleep(2);
          if(moveState)
          {
            flightState=FLIGHT_ARRIVAL_STOCK_POINT;
            flight_log<<"Arrived at stock point."<<endl;
          }
          else
          {
          
          }
        }
        break;
        
        
/********************************************************************************
* 到达抓取点后，开始切换到视觉导航
* 使飞机降至指定高度
*********************************************************************************/
      case FLIGHT_ARRIVAL_STOCK_POINT:
        
        flight_log<<"Visual recogntion starts."<<endl;
        //TODO:开始视觉定位
              
        //if(VisualControl(vehicle, yawFixed, rtkFlightMap[curGrabPoint+3], pCamThreadData))
        if(true)
	      {
          flightState=FLIGHT_RECOGNTION_SUCCESS;
          flight_log<<"Suitable for grabbing."<< endl;
        }
        else
        {
          flight_log << "Not suitable for grabbing." << endl;
          validatedMap[curGrabPoint]=false;
          flightState=FLIGHT_NEW_GRUBBING;
        }
        break;
        
        
/********************************************************************************
* 开始抓取
* 压电传感器和摄像头并用
*********************************************************************************/
      case FLIGHT_RECOGNTION_SUCCESS:
        
        //TODO:开始抓取
        flight_log<<"Begin grabbing-control."<<endl;
        if(!GrabControl(vehicle, yawFixed, rtkFlightMap[curGrabPoint+3], 60000))
        {
          //如果抓取失败，重新起飞进行视觉定位
          flightState=FLIGHT_RECOGNTION_SUCCESS;
          flight_log<<"Grabbing Failed."<<endl;
          //TODO: 抓取失败后的处理

        }
        else
        {
          flightState=FLIGHT_GRABBING_SUCCESS;
          flight_log<<"Grabbing Success."<<endl;
        }  
        
        break;
        

/********************************************************************************
* 抓取成功
* 起飞，飞往摆放点
*********************************************************************************/
      case FLIGHT_GRABBING_SUCCESS:
        
        flight_log<<"Fly to drop point."<<endl;
        moveState=FlightControl(vehicle, yawFixed, rtkFlightMap[1], 15000, 0.15);
        flightState=FLIGHT_ARRIVAL_DROP_POINT;
        flight_log<<"Arrived at drop point."<<endl;
        break;

/********************************************************************************
* 到达摆放点
* 开始摆放
*********************************************************************************/
      case FLIGHT_ARRIVAL_DROP_POINT:
        //TODO:摆放流程
        flight_log<<"Begin to drop."<<endl;
        // 先悬停一段时间，使箱子平稳下来
				sleep(15);
				grabserial.write_cmd('1');  //投放
				sleep(7);
				grabserial.write_cmd('8');  //上拉
				sleep(27);
				grabserial.write_cmd('7');
        // 等待收到‘7’,如果30s没收到，执行下一步
        grabserial.wait_ack('7', 30000);
				//grabserial.write_cmd('0');  //停止上拉
				grabserial.tciflush();			//清空串口输入缓冲区
				flight_log << "Drop done!" <<endl;
        flightState=FLIGHT_DROPPING_SUCCESS;
        cout<<"Dropping Success!"<<endl;
        break;

/********************************************************************************
* 完成一次摆放
* 检查时间、电量、摆放数量
* 决策是否继续
*********************************************************************************/	  
      case FLIGHT_DROPPING_SUCCESS:
        
        numDone+=1;
        
        curTime=time_offset(tv);
        batteryData=vehicle->broadcast->getBatteryInfo().percentage;
        
        flight_log<<"Time:"<<curTime<<endl
          <<"Taskdone:"<<numDone<<endl
          <<"Battery:"<<(int)batteryData<<'%'<<endl;
        
        if((int)batteryData<=BATTERY_THRESHOLD)
        {
          flight_log << "Battery is lower than " << (int)batteryData<<"%, and the UAV is going to return home." <<endl;
        }

        if(numDone<=NUMDONE_THRESHOLD && curTime<TIME_THRESHOLD && (int)batteryData>BATTERY_THRESHOLD)
          flightState=FLIGHT_NEW_GRUBBING;
        else
          flightState=FLIGHT_TASK_DONE;

        break;
      }
    }
  }
}

// target和origin的经纬均为rad
void GetRtkOffset(Vector3lf &deltaNed, Vector3lf &target, Vector3lf &origin)
{
  double deltaLon = target.y - origin.y;
  double deltaLat = target.x - origin.x;
    
  deltaNed.x = deltaLat * C_EARTH;
  deltaNed.y = deltaLon * C_EARTH * cos(target.x);
  deltaNed.z = (target.z - origin.z);
}

void GetRtkFlightMap(Vehicle *vehicle, std::vector<Vector3lf>& _rtkFlightMap)
{
#ifdef __SIMULATION__
  GlobalPosition _pos=vehicle->broadcast->getGlobalPosition();
  Vector3lf tmp;
  tmp.x=_pos.latitude;
  tmp.y=_pos.longitude;
  tmp.z=_pos.height;
  
  tmp.z+=2;
  _rtkFlightMap.push_back(tmp);
  _rtkFlightMap.push_back(tmp);
  _rtkFlightMap.push_back(tmp);
  _rtkFlightMap.push_back(tmp);
  _rtkFlightMap[1].x+=0.000001;
  _rtkFlightMap[3].x+=0.000001;
  _rtkFlightMap[3].y+=0.000001;
#else
  
//TODO:根据地面站协议获取飞行的预设轨迹
  
  _rtkFlightMap.resize(POINT_NUMBER);
  int kc = 0;
  double posi[3]={0.0};
  
  for(kc =0;kc<POINT_NUMBER;kc++)
  {
    getFlowPosition(kc,posi);
    printf("\nposi is %10.6f  %10.6f  %10.3f \n", posi[0], posi[1], posi[2]);
    _rtkFlightMap[kc].y=posi[0]*DEG2RAD;
    _rtkFlightMap[kc].x=posi[1]*DEG2RAD;
    _rtkFlightMap[kc].z=posi[2];
  }
#endif

}

bool GrabControl(Vehicle *vehicle, float32_t yawDesired, Vector3lf &target, int timeoutInMs, float posThresholdInM)
{
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::HORIZONTAL_POSITION | DJI::OSDK::Control::YAW_ANGLE |
                       DJI::OSDK::Control::HORIZONTAL_GROUND | DJI::OSDK::Control::STABLE_ENABLE);
  
  int continueTime = 0;
  int dropState=0;
  char ack;

  float32_t xOffsetRemaining, yOffsetRemaining, zOffsetRemaining;
  float32_t xCmd, yCmd, zCmd;
  int elapsedTime = 0;
  
  double speedFactor_x = 2.0;
  double speedFactor_y = 2.0;
  
  Vector3lf curPosition;
  Vector3lf targetPosition=target;

#ifdef __SIMULATION__
    targetPosition.z-=1;
#else
    targetPosition.z=getHmsOrigin()+GRAB_HEIGHT;
#endif
    cout << "Go down." << endl;
    VerticalControl(vehicle, yawDesired, 0.4, targetPosition, false,30000, 0.05);
    grabserial.write_cmd('2');

  Telemetry::Vector3f cur_v;
  Vector3lf curLocalOffset;
  
  float power_pos_x=CONTROLARGS.grab_xP;
  float power_vel_x=CONTROLARGS.grab_xD;
  
  float power_pos_y=CONTROLARGS.grab_yP;
  float power_vel_y=CONTROLARGS.grab_yD;
  
  const double zP=1;
  const double speedFactor_z=0.25;
  const double speedFactor_min_z=0.01;
  const double xyThreshold=0.5;
  double xyOnZ;
  
  do
  {
		ack=grabserial.read_ack();
    switch(ack)
		{
			case '2': grabserial.write_cmd('3'); break; //抓取
			case '3': dropState=1; break;
			case '4': grabserial.write_cmd('5'); break;
			case '5': grabserial.write_cmd('6'); break;
			case '6': dropState=1; break;
			default: break;
		}

    if (!dropState && elapsedTime >= timeoutInMs)
    {
      flight_log << "Grab Control TimeOut!!!" <<endl;
      break;
    }
    
    if(dropState)
    {
			break;
    }

    curPosition=GetCurPosition(vehicle);
    GetRtkOffset(curLocalOffset, targetPosition, curPosition);
    xOffsetRemaining=curLocalOffset.x;
    yOffsetRemaining=curLocalOffset.y;
    zOffsetRemaining=curLocalOffset.z;
    
    xyOnZ=xOffsetRemaining*xOffsetRemaining+yOffsetRemaining*yOffsetRemaining;
    
    cur_v=vehicle->broadcast->getVelocity();
    
    xCmd = power_pos_x*xOffsetRemaining-power_vel_x*cur_v.x;
    if(std::abs(xCmd)>speedFactor_x)
      xCmd= xCmd > 0 ? speedFactor_x : -speedFactor_x;

    yCmd = power_pos_y*yOffsetRemaining-power_vel_y*cur_v.y;
    if(std::abs(yCmd)>speedFactor_y)
      yCmd= yCmd > 0 ? speedFactor_y : -speedFactor_y;
    

    zCmd=zP*zOffsetRemaining;
    if(std::abs(zCmd)>speedFactor_z)
      zCmd = zCmd >= 0 ? speedFactor_z : -speedFactor_z;
    
    setPosObj(xCmd,yCmd,zCmd);
    setAPIBroadcast(getBroadcastData(vehicle));
    
    DJI::OSDK::Control::CtrlData ctrl_data(ctrl_flag, xCmd, yCmd, zCmd, yawDesired);    
    vehicle->control->flightCtrl(ctrl_data);

    usleep(50000);
    elapsedTime += 50;
    
  }while(1);

  cout << "Go up." << endl;
  VerticalControl(vehicle, yawDesired, 0.3, target, true, 30000, 0.1);
  flight_log << "Grab done!" <<endl;
    
  return true;
}

bool FlightControl(Vehicle *vehicle, float32_t yawDesired, Vector3lf &target, int timeoutInMs, float posThresholdInM)
{
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_POSITION | DJI::OSDK::Control::HORIZONTAL_POSITION | DJI::OSDK::Control::YAW_ANGLE |
		      DJI::OSDK::Control::HORIZONTAL_GROUND | DJI::OSDK::Control::STABLE_ENABLE);
  
  float32_t xOffsetRemaining, yOffsetRemaining, zOffsetRemaining;
  float32_t xCmd, yCmd, zCmd;
  int elapsedTime = 0;
  
  double speedFactor_x = 1.05;
  double speedFactor_min_x = 0.35;
  double speedFactor_y = 1.05;
  double speedFactor_min_y = 0.35;
  double ratio=1.0;
  
  Vector3lf curPosition;
  Telemetry::Vector3f cur_v;
  Vector3lf curLocalOffset;
  
  curPosition=GetCurPosition(vehicle);
  GetRtkOffset(curLocalOffset, target, curPosition);
  xOffsetRemaining=curLocalOffset.x;
  yOffsetRemaining=curLocalOffset.y;
  zOffsetRemaining=curLocalOffset.z;

  if(abs(xOffsetRemaining) > abs(yOffsetRemaining))
  {
    ratio = abs(yOffsetRemaining)/abs(xOffsetRemaining);
    if(ratio < 0.2)
      ratio = 0.2;
    speedFactor_y*=ratio;
    speedFactor_min_y*=ratio;
  }
  else if(abs(xOffsetRemaining) < abs(yOffsetRemaining))
  {
    ratio = abs(xOffsetRemaining)/abs(yOffsetRemaining);
    if(ratio < 0.2)
      ratio = 0.2;
    speedFactor_x*=ratio;
    speedFactor_min_x*=ratio;
  }
  
  float power_pos_x=CONTROLARGS.flight_xP;
  float power_vel_x=CONTROLARGS.flight_xD;
  
  float power_pos_y=CONTROLARGS.flight_yP;
  float power_vel_y=CONTROLARGS.flight_yD;

#ifndef __SIMULATION__
  zCmd=target.z-getHmsOrigin();
#else
  zCmd=target.z;
#endif

  do
  {
    curPosition=GetCurPosition(vehicle);
    GetRtkOffset(curLocalOffset, target, curPosition);
    xOffsetRemaining=curLocalOffset.x;
    yOffsetRemaining=curLocalOffset.y;
    zOffsetRemaining=curLocalOffset.z;
    
    if (elapsedTime >= timeoutInMs)
    {
      flight_log << "flight control TimeOut!!!" << endl;
      break;
    }
    
    cur_v=vehicle->broadcast->getVelocity();
    
    xCmd = power_pos_x*xOffsetRemaining-power_vel_x*cur_v.x;
    yCmd = power_pos_y*yOffsetRemaining-power_vel_y*cur_v.y;
    
    if(std::abs(xCmd)>speedFactor_x)
      xCmd = xCmd > 0 ? speedFactor_x : -speedFactor_x;
    
    if(std::abs(yCmd)>speedFactor_y)
      yCmd = yCmd > 0 ? speedFactor_y : -speedFactor_y;
    
    setPosObj(xCmd,yCmd,zCmd);
    setAPIBroadcast(getBroadcastData(vehicle));
    
    //std::cout << xCmd << '\t' << yCmd << '\t' << zCmd << std::endl;
    DJI::OSDK::Control::CtrlData ctrl_data(ctrl_flag, xCmd, yCmd, zCmd, yawDesired);    
    vehicle->control->flightCtrl(ctrl_data);
    
    usleep(50000);
    elapsedTime += 50;
    
  }while(std::abs(xOffsetRemaining) > posThresholdInM || std::abs(yOffsetRemaining) > posThresholdInM || std::abs(zOffsetRemaining) > posThresholdInM);

  flight_log << "posThresholdInM: " << posThresholdInM <<endl;
  flight_log << "xOffsetRemaining: " << xOffsetRemaining <<endl;
  flight_log << "yOffsetRemaining: " << yOffsetRemaining <<endl;
  flight_log << "zOffsetRemaining: " << zOffsetRemaining <<endl;
  
  return true;
}
// 控制飞机垂直飞行，cmd用于控制垂直起降时是否放绳，cmd=true时，在垂直飞行的同时放绳
bool VerticalControl(Vehicle *vehicle, float32_t yawDesired, float32_t zVelocity, Vector3lf &target, bool cmd, int timeoutInMs, float posThresholdInM)
{
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::HORIZONTAL_POSITION | DJI::OSDK::Control::YAW_ANGLE |
		      DJI::OSDK::Control::HORIZONTAL_GROUND | DJI::OSDK::Control::STABLE_ENABLE);
  
  float32_t xOffsetRemaining, yOffsetRemaining, zOffsetRemaining;
  float32_t xCmd, yCmd, zCmd;
  int elapsedTime = 0;
  int droptime = 29000; //单位：毫秒
  
  Vector3lf curPosition;

  Telemetry::Vector3f cur_v;
  Vector3lf curLocalOffset;
  
  float power_pos_x=CONTROLARGS.grab_xP;
  float power_vel_x=CONTROLARGS.grab_xD;
  
  float power_pos_y=CONTROLARGS.grab_yP;
  float power_vel_y=CONTROLARGS.grab_yD;
  
  const double zP=1;
  const double speedFactor_x = 2.0;
  const double speedFactor_y = 2.0;
  const double speedFactor_z=zVelocity;
  double xyOnZ;

  if(cmd)
    grabserial.write_cmd('9');  //放绳

  do
  {
    curPosition=GetCurPosition(vehicle);
    GetRtkOffset(curLocalOffset, target, curPosition);
    xOffsetRemaining=curLocalOffset.x;
    yOffsetRemaining=curLocalOffset.y;
    zOffsetRemaining=curLocalOffset.z;

    xyOnZ=xOffsetRemaining*xOffsetRemaining+yOffsetRemaining*yOffsetRemaining;

    cur_v=vehicle->broadcast->getVelocity();
    
    xCmd = power_pos_x*xOffsetRemaining-power_vel_x*cur_v.x;
    if(std::abs(xCmd)>speedFactor_x)
      xCmd= xCmd > 0 ? speedFactor_x : -speedFactor_x;

    yCmd = power_pos_y*yOffsetRemaining-power_vel_y*cur_v.y;
    if(std::abs(yCmd)>speedFactor_y)
      yCmd= yCmd > 0 ? speedFactor_y : -speedFactor_y;
    

    zCmd=zP*zOffsetRemaining;
    if(std::abs(zCmd)>speedFactor_z)
      zCmd = zCmd >= 0 ? speedFactor_z : -speedFactor_z;
    
    
    setPosObj(xCmd,yCmd,zCmd);
    setAPIBroadcast(getBroadcastData(vehicle));
    
    DJI::OSDK::Control::CtrlData ctrl_data(ctrl_flag, xCmd, yCmd, zCmd, yawDesired);    
    vehicle->control->flightCtrl(ctrl_data);
    
    usleep(50000);
    elapsedTime += 50;

    if (elapsedTime >= timeoutInMs)
    {
      flight_log << "Vertical Control TimeOut!!!" << endl;
      break;
    }
    
  }while(std::abs(xOffsetRemaining) > posThresholdInM || std::abs(yOffsetRemaining) > posThresholdInM || std::abs(zOffsetRemaining) > posThresholdInM/2);
  
  if(cmd)
  {
    // 设置固定的放绳时间
    //if(elapsedTime < droptime)
    //{
    //  usleep((droptime-elapsedTime)*1000);
    //}
    grabserial.write_cmd('0');  //停止放绳
  }
    
  return true;
}

bool VisualControl(Vehicle *vehicle, float32_t yawDesired, Vector3lf &target, void* data, int timeoutInMs, int detectThrehold)
{
  //ThreadData* const pCamThreadData=(ThreadData*)data;
  
  //if(pCamThreadData->camState!=1)
  //{
  //  flight_log << "Camera is not opened. Visual Control exits." << endl;
  //  return true;
  //}
  
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_POSITION | DJI::OSDK::Control::HORIZONTAL_POSITION | DJI::OSDK::Control::YAW_ANGLE |
		      DJI::OSDK::Control::HORIZONTAL_GROUND | DJI::OSDK::Control::STABLE_ENABLE);

  if(first_drop)
  {
    flight_log << "First drop. Drop directly." << endl;
    first_drop = false;
    return true;
  }
  
  
  //pCamThreadData->count++;
  //pCamThreadData->isEnabled=true;
  

  uint8_t flag=0x81;
  float32_t xOffsetRemaining, yOffsetRemaining, zOffsetRemaining;
  float32_t xCmd, yCmd, zCmd=0;
  int elapsedTime = 0;
  int validatedTime = 0;
  bool isValidated=false;
  
  double speedFactor_x = 2.0;
  double speedFactor_y = 2.0;
  
  Vector3lf curPosition;
  Vector3lf targetPosition=target;
  
  targetPosition.z=getHmsOrigin()+0.20;
  
  Telemetry::Vector3f cur_v;
  Vector3lf curLocalOffset;
  

  float power_pos_x=CONTROLARGS.grab_xP;
  float power_vel_x=CONTROLARGS.grab_xD;
  
  float power_pos_y=CONTROLARGS.grab_yP;
  float power_vel_y=CONTROLARGS.grab_yD;
  
  do
  {
    curPosition=GetCurPosition(vehicle);
    GetRtkOffset(curLocalOffset, targetPosition, curPosition);
    xOffsetRemaining=curLocalOffset.x;
    yOffsetRemaining=curLocalOffset.y;
    zOffsetRemaining=curLocalOffset.z;
        
    cur_v=vehicle->broadcast->getVelocity();
    
    xCmd = power_pos_x*xOffsetRemaining-power_vel_x*cur_v.x;
    if(std::abs(xCmd)>speedFactor_x)
      xCmd= xCmd > 0 ? speedFactor_x : -speedFactor_x;

    yCmd = power_pos_y*yOffsetRemaining-power_vel_y*cur_v.y;
    if(std::abs(yCmd)>speedFactor_y)
      yCmd= yCmd > 0 ? speedFactor_y : -speedFactor_y;
        
    setPosObj(xCmd,yCmd,zCmd);
    setAPIBroadcast(getBroadcastData(vehicle));
    
    DJI::OSDK::Control::CtrlData ctrl_data(ctrl_flag, xCmd, yCmd, zCmd, yawDesired);    
    vehicle->control->flightCtrl(ctrl_data);
    usleep(50000);

    //pthread_mutex_lock(&pCamThreadData->plock);
    //isValidated=pCamThreadData->isValidated;
    //pthread_mutex_unlock(&pCamThreadData->plock);
    
    if(isValidated)
    {
      validatedTime += 50;
    }
    elapsedTime += 50;
    
  }while(elapsedTime<timeoutInMs);
  
  //pCamThreadData->isEnabled=false;
  
  flight_log << "validatedTime: " << validatedTime << "ms" <<endl;
  
  if(validatedTime >= detectThrehold)
    return true;
  else
    return false;
}

void printPos(Vehicle *vehicle)
{
  Vector3lf pos=GetCurPosition(vehicle);
  
  flight_log<<pos.x<<endl
  <<pos.y<<endl
  <<pos.z<<endl;
}

