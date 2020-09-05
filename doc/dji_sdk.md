---
title: DJI SDK笔记
date: 2018-08-31 10:53:15
tags:
---

## OSDK介绍

### 什么时候适合用OSDK

* 精确轨迹飞行 
* 无远程控制情况下飞行
* 集成第三方传感器、执行机构和通信系统的情况下

### 硬件介绍

https://developer.dji.com/onboard-sdk/documentation/introduction/osdk-hardware-introduction.html  

### 功能概述

#### 飞控

大疆搭载的SDK允许多种方式来控制飞机的飞行：

* **Attitude Control**：底层姿态命令
* **Velocity Control**：底层速度命令
* **Position Control**：底层位置命令
* **Missions**：方便、易于实现对飞机的高层控制。例如，可以使用路径点任务执行定义的飞行路径。

#### 飞行数据测量记录

可通过OSDK以可达200Hz的频率获取传感器数据和飞机状态数据：

惯性传感器数据：GPS、RTK、Compass、Barometer  
飞机状态数据：速度、姿态、万向节状态、电池状态、四元数、线加速度、角速率  

## 飞控

### 飞行

#### 电机控制

在大疆机上的SDK中，可以通过api开启和关闭电机。只有当飞机没有飞行时，电机才能关闭。如果有IMU或罗盘校准错误，或者IMU仍在预热，电机将不会开启。  

#### 开始和结束飞行

飞机起飞和降落可以通过大疆车载SDK中的api实现自动化。当飞机在离地面12米（4英尺）时，起飞被认为是完成了。自动起飞只能在发动机关闭时启动。  

当一个自动着陆命令被发送时，飞机将降落在它的当前位置并着陆。  

## 模式标志字节说明
### 模式标志字节
模式标志字节代表不同模式的配置。因为多旋翼的结构特点，飞行控制的时候，要把控制信息分解成三部分，竖直、水平和偏航，每个部分都有几种选择，通过模式标志字节进行组合。模式标志字节的定义如下。
<table><tr><th>名称</th><th>大小</th><th>说明</th></tr><tr>
<td rowspan="5">模式标志字节<td>bit7:6</td><td>0b00：HORI_ATTI_TILT_ANG<br>0b01：HORI_VEL<br>0b10：HORI_POS</td></tr><tr><td>bit5:4</td><td>0b00：VERT_VEL<br>0b01：VERT_POS<br>0b10：VERT_THRUST</td></tr><tr><td>bit3</td><td>0b0: YAW_ANG<br>0b1: YAW_RATE</td></tr><tr><td>bit2:1</td><td>0b00：水平方向坐标系为 Ground 系<br>0b01：水平方向坐标系为 Body 系</td></tr><tr><td>bit0</td><td>0b0：非增稳模式<br>0b1：增稳模式</td></tr></table>

### 控制模式

我们建议用户在室内环境中，如果没有安装Gudiance或者飞行高度超过3m时，不要使用竖直方向的位置控制，因为室内环境中气压计读数不准，影响控制器的表现。  

>备注：部分控制模式有进入条件限制：  

>- 当且仅当GPS信号正常（health\_flag >=3）时，才可以使用水平**位置**控制（HORI_POS）相关的控制指令  
>- 当GPS信号正常（health\_flag >=3），或者Gudiance系统正常工作（连接安装正确）时，可以使用水平**速度**控制（HORI_VEL）相关的控制指令  
<table><tr><th>类别</th><th>模式</th><th>说明</th></tr><tr><td rowspan="3">竖直方向</td><td>VERT_POS</td><td>垂直方向上控制的是位置，输入的控制量为相对地面的高度</td></tr><tr><td>VERT_VEL</td><td>垂直方向上控制的是速度</td></tr><tr><td>VERT_THRUST</td><td>垂直方向上控制的是油门百分比(0-100)（危险，请小心使用）</td></tr><tr><td rowspan="3">水平方向</td><td>HORI_ATTI_TILT_ANG*</td><td>水平方向控制的是飞行器水平倾角，根据坐标系确定水平分量</td></tr><tr><td>HORI_POS**</td><td>水平方向控制的是飞行器飞行距离**，根据坐标系确定水平分量</td></tr><tr><td>HORI_VEL</td><td>水平方向控制的是飞行器速度，根据坐标系确定水平分量</td></tr><tr><td rowspan="2">偏航</td><td>YAW_ANG</td><td>偏航控制一个 Ground 坐标系下的目标角度。此模式下，飞控会将YAW坐标系强制为Ground系</td></tr><tr><td>YAW_RATE</td><td>偏航控制目标角速度</td></tr></table>

>**HORI_POS模式的输入量是相对位置的净输入量。净输入量通过GPS或Guidance等传感器获得位置移动的反馈信息，与输入作差后得到。为完成飞行的位置控制，需要连续获得反馈并发送该命令，以达到平稳的控制效果。**

### 模式的组合 

姿态控制精度大约为0.5 度，速度控制精度大约为0.2 m/s。

|模式编号|组合形式|输入数值范围<br>(VERT/HORI/YAW)|模式标志字节|  
|-------|-----|-----------------------------------------|--------|  
|1|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-30 度 ~ 30 度<br>-180 度 ~ 180 度|0b00000xxy|
|2|VERT_VEL<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-30 度 ~ 30 度<br>-100 度/s ~ 100 度/s|0b00001xxy|
|3|VERT_VEL<br>HORI_VEL<br>YAW_ANG|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-180 度 ~ 180 度|0b01000xxy|
|4|VERT_VEL<br>HORI_VEL<br>YAW_RATE|-4 m/s ~ 4 m/s<br>-10 m/s ~ 10 m/s<br>-100 度/s ~ 100 度/s|0b01001xxy|
|5|VERT_VEL<br>HORI_POS<br>YAW_ANG|-4 m/s ~ 4 m/s<br>米为单位的相对位置，数值无限制<br>-180 度 ~ 180 度|0b10000xxy|
|6|VERT_VEL<br>HORI_POS<br>YAW_RATE|-4 m/s ~ 4 m/s<br>米为单位的相对位置，数值无限制<br>-100 度/s ~ 100 度/s|0b10001xxy|
|7|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|0m 到最大飞行高度<br>-30 度 ~ 30 度<br>-180 度 ~ 180 度|0b00010xxy|
|8|VERT_POS<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|0m 到最大飞行高度<br>-30 度 ~ 30 度<br>-100 度/s ~ 100 度/s|0b00011xxy|
|9|VERT_POS<br>HORI_VEL<br>YAW_ANG|0m 到最大飞行高度<br>-10 m/s ~ 10 m/s<br>-180 度 ~ 180 度|0b01010xxy|
|10|VERT_POS<br>HORI_VEL<br>YAW_RATE|0m 到最大飞行高度<br>-10 m/s ~ 10 m/s<br>-100 度/s ~ 100 度/s|0b01011xxy|
|11|VERT_POS<br>HORI_POS<br>YAW_ANG|0m 到最大飞行高度<br>米为单位的相对位置，数值无限制<br>-180 度 ~ 180 度|0b10010xxy|
|12|VERT_POS<br>HORI_POS<br>YAW_RATE|0m 到最大飞行高度<br>米为单位的相对位置，数值无限制<br>-100 度/s ~ 100 度/s|0b10011xxy|
|13|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_ANG|10 ~ 100 （危险，请小心使用）<br>-30 度 ~ 30 度<br>-180 度 ~ 180 度|0b00100xxy|
|14|VERT_THRUST<br>HORI_ATTI_TILT_ANG<br>YAW_RATE|10 ~ 100（危险，请小心使用）<br>-30 度 ~ 30 度<br>-100 度/s ~ 100 度/s|0b00101xxy|
 
>xx表示水平方向坐标系的控制位，00表示Ground系，01表示Body系    
>y表示增稳模式的控制位，0表示非增稳模式，1表示增稳模式，增稳模式只作用于水平方向。

## 飞行数据说明
### 飞行数据

<table><tr><th>状态包</th><th>状态包字段</th><th>数据段类型</th><th>描述</th><th>单位</th><th>默认频率</th></tr><tr><td rowspan="3">时间戳</td><td>time</td><td>uint32_t</td><td>400hz时间戳</td><td>1/400s</td><td rowspan="3">100Hz</td> </tr><tr> <td>asr_ts</td><td>uint32_t</td> <td>ns级时间戳</td> <td>ns</td> </tr> <tr> <td>sync_flag</td> <td>uint8_t</td> <td>同步信号标志位</td> <td>---</td> </tr> <tr> <td rowspan="4">姿态四元数</td> <td>q0</td> <td>float32</td> <td rowspan="4">姿态四元数<br>从Ground坐标系到Body坐标系</td> <td rowspan="4">---</td> <td rowspan="4">100Hz</td> </tr> <tr> <td>q1</td> <td>float32</td> </tr> <tr> <td>q2</td> <td>float32</td> </tr> <tr> <td>q3</td> <td>float32</td> </tr> <tr> <td rowspan="3">加速度</td> <td>agx</td> <td>float32</td> <td rowspan="3">加速度（原始值/融合值）</td> <td rowspan="3">融合值: m/s<sup>2</sup><br>原始值: G</td> <td rowspan="3">100Hz</td> </tr> <tr> <td>agy</td> <td>float32</td> </tr> <tr> <td>agz</td> <td>float32</td> </tr> <tr> <td rowspan="4">速度</td> <td>vgx</td> <td>float32</td> <td rowspan="3">Ground系下的速度</td> <td rowspan="3">m/s</td> <td rowspan="4">100Hz</td> </tr> <tr> <td>vgy</td> <td>float32</td> </tr> <tr> <td>vgz</td> <td>float32</td> </tr> <tr> <td>vgstatus</td> <td>uint8_t</td> <td>速度信息状态字节<ul> <li>bit 0：数据有效位</li> <ul>0：速度数据无效</ul> <ul>1：速度数据有效</ul> <li>bit 1:7 ：保留</li> </ul></td> <td>---</td> </tr> <tr> <td rowspan="3">角速度</td> <td>wx</td> <td>float32</td> <td rowspan="3">角速度（原始值/融合值）</td> <td rowspan="3">rad/s</td> <td rowspan="3">100Hz</td> </tr> <tr> <td>wy</td> <td>float32</td> </tr> <tr> <td>wz</td> <td>float32</td> </tr> <tr> <td rowspan="5">GPS及高度</td> <td>longti</td> <td>double</td> <td rowspan="2">GPS 位置</td> <td rowspan="2">rad</td> <td rowspan="5">100Hz</td> </tr> <tr> <td>lati</td> <td>double</td> </tr> <tr> <td>alti</td> <td>float32</td> <td>气压高度（原始值/融合值）</td> <td>m</td> </tr> <tr> <td>height</td> <td>float32</td> <td>对地高度（原始值/融合值）</td> <td>m</td> </tr> <tr> <td>health_flag</td> <td>uint8_t</td> <td>GPS 健康度 </td> <td>0-5, 5 为最好</td> </tr> <tr> <td rowspan="8"> GPS详细信息（只适用于A3）</td> <td>date</td> <td>uint32_t</td> <td>日期</td> <td>yy-mm-dd</td> <td rowspan="8">50Hz</td> </tr> <tr> <td>time</td> <td>uint32_t</td> <td>时间</td> <td>hh-mm-ss</td> </tr> <tr> <td>longitude</td> <td>int32_t</td> <td>经度</td> <td>degree*10^7</td> </tr> <tr> <td>latitude</td> <td>int32_t</td> <td>纬度</td> <td>degree*10^7</td> </tr> <tr> <td>AMSL</td> <td>int32_t</td> <td>海拔高度</td> <td>mm</td> </tr> <tr> <td>vel_N</td> <td>float32</td> <td>北方向速度</td> <td>cm/s</td> </tr> <tr> <td>vel_E</td> <td>float32</td> <td>东方向速度</td> <td>cm/s</td> </tr> <tr> <td>vel_D</td> <td>float32</td> <td>地方向速度</td> <td>cm/s</td> </tr> <tr> <td rowspan="11"> RTK详细信息（只适用于A3）</td> <td>date</td> <td>uint32_t</td> <td>日期</td> <td>yy-mm-dd</td> <td rowspan="11">50Hz</td> </tr> <tr> <td>time</td> <td>uint32_t</td> <td>时间</td> <td>hh-mm-ss</td> </tr> <tr> <td>longitude_RTK</td> <td>double</td> <td>RTK测量经度</td> <td>degree</td> </tr> <tr> <td>latitude_RTK</td> <td>double</td> <td>RTK测量纬度</td> <td>degree</td> </tr> <tr> <td>AMSL_RTK</td> <td>float32</td> <td>RTK测量海拔高度</td> <td>m</td> </tr> <tr> <td>vel_N</td> <td>float32</td> <td>北方向速度</td> <td>cm/s</td> </tr> <tr> <td>vel_E</td> <td>float32</td> <td>东方向速度</td> <td>cm/s</td> </tr> <tr> <td>vel_D</td> <td>float32</td> <td>地方向速度</td> <td>cm/s</td> </tr> <tr> <td>yaw</td> <td>int16_t</td> <td>天线基线与正南夹角</td> <td>degree</td> </tr> <tr> <td>position_flag</td> <td>uint8_t</td> <td>定位标志位</td> <td></td> </tr> <tr> <td>yaw_flag</td> <td>uint8_t</td> <td>偏航标志位</td> <td>---</td> </tr> <tr> <td rowspan="3">磁感计</td> <td>mx</td> <td>int16_t</td> <td rowspan="3">磁感计数值</td> <td rowspan="3">磁感计数值</td> <td rowspan="3">0Hz</td> </tr> <tr> <td>my</td> <td>int16_t</td> </tr> <tr> <td>mz</td> <td>int16_t</td> </tr>  <tr> <td rowspan="6">遥控器通道</td> <td>roll</td> <td>int16_t</td> <td>Roll 数值</td> <td rowspan="6">---</td> <td rowspan="6">50Hz</td> </tr> <tr> <td>pitch</td> <td>int16_t</td> <td>Pitch 数值</td> </tr> <tr> <td>yaw</td> <td>int16_t</td> <td>Yaw 数值</td> </tr> <tr> <td>throttle</td> <td>int16_t</td> <td>Throttle 数值</td> </tr> <tr> <td>mode</td> <td>int16_t</td> <td>Mode 数值（模式选择开关）</td> </tr> <tr> <td>gear</td> <td>int16_t</td> <td>Gear 数值（返航键外圈拨杆）</td> </tr>  <tr> <td rowspan="4">云台姿态</td> <td>roll</td> <td>float32</td> <td rowspan="3">云台在Ground 坐标系下的姿态</td> <td rowspan="3">º</td> <td rowspan="4">50Hz</td> </tr> <tr> <td>pitch</td> <td>float32</td> </tr> <tr> <td>yaw</td> <td>float32</td> </tr> <tr> <td>limit_byte</td> <td>uint8_t</td> <td>限位标志<ul> <li>bit 0: Pitch限位标志</li> <li>bit 1: Roll限位标志</li> <li>bit 2: Yaw限位标志</li> <li>bit 3:7 保留</li></ul> <td>---</td> </tr> <tr> <td>飞行状态</td> <td>status</td> <td>uint8_t</td> <td>飞行状态</td> <td>---</td> <td>10Hz</td> </tr>  <tr> <td>电量</td> <td>status</td> <td>uint8_t</td> <td>剩余电量百分比</td> <td>%</td> <td>1Hz</td> </tr>  <tr> <td rowspan="2">控制信号源</td> <td>cur_mov_control_mode</td> <td>uint8_t</td> <td>当前飞机模式值</td> <td>---</td> <td rowspan="2">0Hz</td> </tr>  <tr> <td>status</td> <td>uint8_t</td> <td>控制设备<ul> <li>bit 0:2 ：控制设备</li> <ul>0b000 ：遥控器</ul> <ul>0b001 ：移动设备</ul> <ul>0b010 ：机载设备</ul> <li>bit 3 ：机载设备控制命令标志位</li> <ul>0：未命令</ul> <ul>1：已命令</ul> <li>bit 4 ：虚拟遥控标志位</li> <ul>0：未启用</ul> <ul>1：已启用</ul> <li>bit 5:7 ：保留</li> </ul></td> <td>---</td> </tr></table>

## 编程

0x91 HORI_POS VERT_POS YAW_ANG Ground系 增稳模式  
0x99 HORI_POS VERT_POS  YAW_RATE Ground系 增稳模式  

## 仿真

绿灯双闪：视觉导航模式，在仿真时出现此状态一般为错误。  

### 仿真步骤
1. 断开USB仿真线
2. 打开PC端仿真软件
3. 打开飞机电源
4. 连接USB仿真线
5. 开始仿真
