#pragma once
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <string.h>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>  // 串口操作相关库
#include <stdbool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>

using namespace std;

#define SEND_DATA_CHECK   1     // 标志位，发送端做校验位
#define READ_DATA_CHECK   0     // 标志位，接收端做校验位
#define FRAME_HEADER      0X7B  // 帧头，和下位机一致
#define FRAME_TAIL  0X7D // 帧尾
#define FRAME_HEADER_ARM      0XAA  // 帧头，和下位机一致
#define FRAME_TAIL_ARM  0XBB // 帧尾
#define RECEIVE_DATA_SIZE        24 // 下位机发过来的数据的长度
#define SEND_DATA_SIZE            16 // ROS发给下位机的数据的长度 考虑到时效性短尽短
#define DEFAULT_MODE  1
#define FOLLOWER 2
#define PI                3.1415926f // 圆周率
#define GYROSCOPE_RATIO 0.00026644f // 陀螺仪原始数据换成弧度单位
#define ACCEL_RATIO    16384.0f   // 加速度换算比例，量程±2g，重力加速度定义为1g等于9.8米每平方秒。

extern sensor_msgs::msg::Imu Mpu6050;

// 协方差矩阵
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
                                         0, 1e-3, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e3};

// 速度/位置结构体
typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
} Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
    short accele_x_data;
    short accele_y_data; 
    short accele_z_data; 
    short gyros_x_data; 
    short gyros_y_data; 
    short gyros_z_data; 
} MPU6050_DATA;

typedef struct _SEND_DATA_  
{
    uint8_t tx[SEND_DATA_SIZE];
    float X_speed;        
    float Y_speed;           
    float Z_speed;         
    unsigned char Frame_Tail;  // 1个字节 帧尾 校验位
} SEND_DATA;

typedef struct _SEND_DATA2_  
{
    uint8_t tx[SEND_DATA_SIZE - 5];
    float X_speed;       
    float Y_speed;           
    float Z_speed;         
    unsigned char Frame_Tail;  // 1个字节 帧尾 校验位
} SEND_DATA2;

typedef struct _RECEIVE_DATA_     
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header; // 1个字节 帧头
    float X_speed;  
    float Y_speed;  
    float Z_speed;  
    float Power_Voltage;    
    unsigned char Frame_Tail;  // 1个字节 帧尾 校验位
} RECEIVE_DATA;

// 定义 TurnOnRobot 类
class TurnOnRobot : public rclcpp::Node
{
public:
    TurnOnRobot();  // 构造函数
    ~TurnOnRobot(); // 析构函数
    void Control(); // 循环控制代码
    //serial::Serial Stm32_Serial;  // 串口对象

    // 串口操作相关的声明
    void open_serial_port();
    ssize_t read_from_serial_port(uint8_t *buffer, size_t size);
    void write_to_serial_port(uint8_t *data, size_t size);
    void close_serial_port();

private:
    // 回调函数声明
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
    void joint_states_Callback(const sensor_msgs::msg::JointState::SharedPtr arm_joint);
    void arm_teleop_Callback(const sensor_msgs::msg::JointState::SharedPtr arm_joint);
    bool Get_Sensor_Data();
    short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);
    float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);

    // 发布器
    void Publish_Odom();
    void Publish_Pose();
    void Publish_ImuSensor();
    void Publish_Voltage();
    void init_joint_states();

    unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);  // 校验函数

    // 机器人的位置和速度
    Vel_Pos_Data Robot_Pos;
    Vel_Pos_Data Robot_Vel;

    // ROS2相关变量
    rclcpp::Time _Now, _Last_Time;  // 时间相关
    float Sampling_Time;  // 采样时间
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_Sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_teleop_Sub;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;
    
    tf2_ros::TransformBroadcaster odom_broadcaster;
    
    // 电源电压
    float Power_voltage;
    
    // 串口相关
    int serial_baud_rate;
    string usart_port_name, robot_frame_id, odom_frame_id, smoother_cmd_vel;
    int joint_num;

    // 串口数据结构
    RECEIVE_DATA Receive_Data;  // 接收结构体
    SEND_DATA Send_Data;  // 发送结构体 16位
    SEND_DATA2 Send_Data2;  // 发送结构体 11位
    MPU6050_DATA Mpu6050_Data;

    int serial_fd;  // 串口文件描述符
};
