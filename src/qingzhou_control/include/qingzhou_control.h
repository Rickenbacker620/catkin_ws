

#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include <iostream>
#include <math.h>
#include <string.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>

//lib
#include <serial/serial.h>
#include <time.h>

#define CARL 0.285
#define CARW 0.35
#define PI 3.14159265
using namespace std;
using namespace boost::asio;
using namespace boost;

#pragma pack(push)
#pragma pack(1)
typedef union
{
  uint8_t raw[33];
  struct
  {
    uint16_t header;
    uint8_t len;
    int re_Encoder_Left;
    int re_Encoder_Right;
    int Voltage;
    short accelX;
    short accelY;
    short accelZ;
    short gyroX;
    short gyroY;
    short gyroZ;
    short magX;
    short magY;
    short magZ;
  } data;
} carInfo;
#pragma pack(pop)

typedef struct
{
  int TargetAngleDir;   //期望转向角度符号 0:直行；0x10:左转；0x20:右转；
  int TargetAngle;      //期望角度
  int TargetSpeed;      //期望速度
  int TargetModeSelect; //模式选择
  int TargetShiftPosition;
  bool control;
} SmartcarControl;

class Actuator
{
public:
  Actuator();
  ~Actuator();

  void run();

public:
  int m_baudrate;
  int m_deviceName;
  int m_runningmode; //运行模式

  std::string m_serialport; //对应USB端口

  int encoderLeft;         //左编码器
  int encoderRight;        //右编码器
  int calibrate_lineSpeed; //标定线速度

  float ticksPerMeter;  //一米脉冲数
  float ticksPer2PI;    //每圈脉冲数
  float linearSpeed;    //线速度
  float angularSpeed;   //角速度
  float batteryVoltage; //电池电压

  double x;  //x坐标
  double y;  //y坐标
  double th; //角度

  ros::Time current_time_;
  ros::Time last_time_;
  double delta_time_;

  double accelX, accelY, accelZ; //加速度
  double gyroX, gyroY, gyroZ;    //角速度
  double magX, magY, magZ;       //磁力计

  double detDistance;
  double detTh; //计算距离和计算角度角度
  double detEncode;

  long long LeftticksPerMeter = 0;  //左轮编码器每米脉冲数
  long long rightticksPerMeter = 0; //右轮编码器每米脉冲数
  long long LeftticksPer2PI = 0;    //左轮每圈编码器脉冲数
  long long rightticksPer2PI = 0;   //右轮每圈编码器脉冲数

  SmartcarControl carParasControl; //根据之前定义的结构体，声明小车控制数据，未使用？

  serial::Serial ser; //声明串口ser

  //msg
  SmartcarControl moveBaseControl;
  std_msgs::Float32 currentBattery; //当前电压

  //订阅话题
  ros::Subscriber sub_imudata;
  ros::Subscriber sub_move_base; //订阅move_base

  //发布话题
  ros::Publisher pub_actuator;
  ros::Publisher pub_odom;    //发布odom
  ros::Publisher pub_imu;     //发布imu
  ros::Publisher pub_mag;     //发布磁力计
  ros::Publisher pub_battery; //发布电池

  void processBattery();
  void processImu(); //发布9250函数
  void processOdom();

  void recvCarInfoKernel(); //接收下位机发来数据函数
  void sendCarInfoKernel(); //发送小车数据到下位机函数

  void callback_imuData(const sensor_msgs::Imu::ConstPtr &msg);       //imu数据回调函数 未使用
  void callback_move_base(const geometry_msgs::Twist::ConstPtr &msg); //move_base回调数据
};

#endif // NODE_EXAMPLE_TALKER_H
