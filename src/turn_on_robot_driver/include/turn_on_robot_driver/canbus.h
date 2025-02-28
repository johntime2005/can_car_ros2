#ifndef _CANBUS_H
#define _CANBUS_H

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

using namespace std;

#if defined(_WIN32) || defined(__CYGWIN__) || defined(_WIN32_WCE)
#define LIBUSB_CALL __stdcall
#else
#define LIBUSB_CALL
#endif

typedef unsigned int u32;
typedef unsigned char u8;

typedef struct _Can_Msg {
    u32             ID;
    u32             TimeStamp;
    char            FrameType;
    char            DataLen;
    unsigned char            Data[8];
    char            ExternFlag;
    char            RemoteFlag;
    char            BusSatus;
    char            ErrSatus;
    char            TECounter;
    char            RECounter;
} Can_Msg, *P_Can_Msg;

typedef struct _Can_Status {
    char            BusSatus;
    char            ErrSatus;
    char            TECounter;
    char            RECounter;
} Can_Status, *P_Can_Status;

typedef struct _Can_Config {
    u32             baudrate;
    u32             configs;
    u32             model;
} Can_Config, *P_Can_Config;

#ifdef __cplusplus
extern "C" {
#endif

int LIBUSB_CALL  Reg_HotPlug_Func(void(*pfunc)(void));
int LIBUSB_CALL  CAN_ScanDevice(void);
int LIBUSB_CALL  CAN_OpenDevice(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_CloseDevice(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_Init(u32 devNum, u32 chNum, P_Can_Config pInitConfig);
int LIBUSB_CALL  CAN_SetFilter(u32 devNum, u32 chNum, char namber, char type, u32 ftID, u32 ftMask, char enable);
int LIBUSB_CALL  CAN_Reset(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_GetReceiveNum(u32 devNum, u32 chNum);
int LIBUSB_CALL  CAN_Transmit(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 items, u32 timeou);
int LIBUSB_CALL  CAN_Receive(u32 devNum, u32 chNum, P_Can_Msg canmsg, u32 Len, u32 timeou);
int LIBUSB_CALL  CAN_GetStatus(u32 devNum, u32 chNum, P_Can_Status status);
#ifdef __cplusplus
}
#endif

// Macro definition
#define SEND_DATA_CHECK   1          
#define READ_DATA_CHECK   0          
#define FRAME_HEADER      0X7B       
#define FRAME_TAIL        0X7D       
#define RECEIVE_DATA_SIZE 24         
#define SEND_DATA_SIZE    11         
#define PI                3.1415926f  

#define GYROSCOPE_RATIO   0.00026644f
#define ACCEl_RATIO       1671.84f

extern sensor_msgs::msg::Imu Mpu6050;

// 协方差矩阵定义
const double odom_pose_covariance[36] = {
    1e-3,    0,    0,   0,   0,    0, 
      0, 1e-3,    0,   0,   0,    0,
      0,    0,  1e6,   0,   0,    0,
      0,    0,    0, 1e6,   0,    0,
      0,    0,    0,   0, 1e6,    0,
      0,    0,    0,   0,   0,  1e3 
};

const double odom_pose_covariance2[36] = {
    1e-9,    0, 1e-9,   0,   0,    0, 
      0, 1e-3,    0,   0,   0,    0,
      0,    0,  1e6,   0,   0,    0,
      0,    0,    0, 1e6,   0,    0,
      0,    0,    0,   0, 1e6,    0,
      0,    0,    0,   0,   0, 1e-9 
};

const double odom_twist_covariance[36] = {
    1e-3,    0,    0,   0,   0,    0, 
      0, 1e-3,    0,   0,   0,    0,
      0,    0,  1e6,   0,   0,    0,
      0,    0,    0, 1e6,   0,    0,
      0,    0,    0,   0, 1e6,    0,
      0,    0,    0,   0,   0,  1e3 
};

const double odom_twist_covariance2[36] = {
    1e-9,    0, 1e-9,   0,   0,    0, 
      0, 1e-3,    0,   0,   0,    0,
      0,    0,  1e6,   0,   0,    0,
      0,    0,    0, 1e6,   0,    0,
      0,    0,    0,   0, 1e6,    0,
      0,    0,    0,   0,   0, 1e-9 
};

class turn_on_robot : public rclcpp::Node
{
public:
    turn_on_robot() : Node("turn_on_robot"), Sampling_Time(0.01)
    {
        // 创建订阅者（订阅 cmd_vel 话题）
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, std::placeholders::_1)
        );

        // 创建发布者
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        voltage_publisher_ = this->create_publisher<std_msgs::msg::Float32>("voltage", 10);

        _Last_Time = this->now();
    }

    ~turn_on_robot() {}

    void Control();   // 循环控制代码

private:
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
    void Publish_Odom();      // 发布里程计话题
    // void Publish_ImuSensor(); // 发布IMU传感器话题（如需要可实现）
    void Publish_Voltage();   // 发布电压话题
    bool Get_Sensor_Data();   
    bool Get_Sensor_Data_New();
    unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);
    float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);
    void print_frame(P_Can_Msg pCan_Msg);
    void calculate_speed(int speed, int rad, unsigned char *data);
    void move(int speed, int rad);
    void timer_callback();

    // 时间、采样周期变量
    rclcpp::Time _Now, _Last_Time;
    float Sampling_Time;

    // ROS2 订阅者和发布者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher_;

    // 其它变量
    string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
    int serial_baud_rate;

    // ROS 向下位机发送数据结构体
    typedef struct _SEND_DATA_ {
        uint8_t tx[SEND_DATA_SIZE];
        float   X_speed;
        float   Y_speed;
        float   Z_speed;
        unsigned char Frame_Tail;
    } SEND_DATA;

    // 下位机接收数据结构体
    typedef struct _RECEIVE_DATA_ {
        uint8_t rx[RECEIVE_DATA_SIZE];
        uint8_t Flag_Stop;
        unsigned char Frame_Header;
        float   X_speed;
        float   Y_speed;
        float   Z_speed;
        float   Power_Voltage;
        unsigned char Frame_Tail;
    } RECEIVE_DATA;

    RECEIVE_DATA Receive_Data;
    SEND_DATA Send_Data;

    // 机器人位置、速度数据结构体
    typedef struct __Vel_Pos_Data_ {
        float X;
        float Y;
        float Z;
    } Vel_Pos_Data;

    Vel_Pos_Data Robot_Pos;
    Vel_Pos_Data Robot_Vel;
};

#endif