#include "armdriver/armdriver.hpp"
#include "armdriver/quaternion.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <cstdint>

sensor_msgs::msg::Imu Mpu6050;  // 实例化IMU对象

/**************************************
Date: May 31, 2020
Function: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // ROS2 初始化
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "wheeltec_arm_six node has turned on");  // ROS2 打印
    auto robot_control = std::make_shared<TurnOnRobot>();  // 实例化对象
    robot_control->Control();  // 循环执行数据采集和发布操作
    rclcpp::shutdown();  // ROS2 关闭
    return 0;
}

/**************************************
Date: May 31, 2020
Function: 构造函数, 只执行一次，用于初始化
***************************************/
TurnOnRobot::TurnOnRobot()
    : Node("wheeltec_arm_six"), Sampling_Time(0), odom_broadcaster(*this), Power_voltage(0.0)
{
    // 初始化类成员
    memset(&Robot_Pos, 0, sizeof(Robot_Pos));
    memset(&Robot_Vel, 0, sizeof(Robot_Vel));
    memset(&Receive_Data, 0, sizeof(Receive_Data));
    memset(&Send_Data, 0, sizeof(Send_Data));
    memset(&Send_Data2, 0, sizeof(Send_Data2));
    memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

    // 获取参数
    declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0");
    declare_parameter<int>("serial_baud_rate", 115200);
    declare_parameter<std::string>("smoother_cmd_vel", "/smoother_cmd_vel");
    declare_parameter<std::string>("robot_frame_id", "base_link");
    declare_parameter<std::string>("odom_frame_id", "odom_combined");
    declare_parameter<int>("joint_num", 4);

    // 读取参数
    get_parameter("usart_port_name", usart_port_name);
    get_parameter("serial_baud_rate", serial_baud_rate);
    get_parameter("smoother_cmd_vel", smoother_cmd_vel);
    get_parameter("robot_frame_id", robot_frame_id);
    get_parameter("odom_frame_id", odom_frame_id);
    get_parameter("joint_num", joint_num);

    // 发布话题
    voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu", 20);
    pose_publisher = create_publisher<geometry_msgs::msg::Pose>("pose", 20);

    // 订阅话题
    Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 100, std::bind(&TurnOnRobot::Cmd_Vel_Callback, this, std::placeholders::_1));
    joint_state_Sub = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 100, std::bind(&TurnOnRobot::joint_states_Callback, this, std::placeholders::_1));
    arm_teleop_Sub = create_subscription<sensor_msgs::msg::JointState>(
        "arm_teleop", 100, std::bind(&TurnOnRobot::arm_teleop_Callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Data ready");

    // 初始化串口
    try
    {
        open_serial_port();  // 打开串口
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Unable to open serial port: %s", e.what());
    }

    init_joint_states();  // 开机过程机械臂运动到预设位置
}

/**************************************
Date: May 31, 2020
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
TurnOnRobot::~TurnOnRobot()
{
    Send_Data2.tx[0] = FRAME_HEADER;  // 帧头 固定值
    Send_Data2.tx[1] = 0;  // 产品型号
    Send_Data2.tx[2] = 0;  // 机器人使能控制标志位
    Send_Data2.tx[4] = 0;  // 机器人x轴的目标线速度
    Send_Data2.tx[3] = 0;  // 机器人x轴的目标线速度
    Send_Data2.tx[6] = 0;  // 机器人y轴的目标线速度
    Send_Data2.tx[5] = 0;  // 机器人y轴的目标线速度
    Send_Data2.tx[8] = 0;  // 机器人z轴的目标角速度
    Send_Data2.tx[7] = 0;  // 机器人z轴的目标角速度

    Send_Data2.tx[9] = Check_Sum(9, SEND_DATA_CHECK);  // 帧尾校验位
    Send_Data2.tx[10] = FRAME_TAIL;  // 数据的最后一位是帧尾（固定值）

    try
    {
        write_to_serial_port(Send_Data2.tx, sizeof(Send_Data2.tx));  // 向串口发送数据
        RCLCPP_INFO(get_logger(), "New control command");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Unable to send data through serial port");
    }

    close_serial_port();  // 关闭串口
    RCLCPP_INFO(get_logger(), "Shutting down");
}

/**************************************
Date: May 31, 2020
Function: 打开串口
***************************************/
void TurnOnRobot::open_serial_port()
{
    // 打开串口设备
    serial_fd = open(usart_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);  // 打开串口，读写模式，非阻塞
    if (serial_fd == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", usart_port_name.c_str());
        throw std::runtime_error("Failed to open serial port " + usart_port_name);
    }

    RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", usart_port_name.c_str());

    // 获取当前串口配置
    struct termios serial_options;
    if (tcgetattr(serial_fd, &serial_options) != 0)  // 获取串口属性
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes");
        close(serial_fd);
        throw std::runtime_error("Failed to get serial port attributes");
    }

    // 配置串口波特率
    cfsetispeed(&serial_options, B115200);  // 设置输入波特率
    cfsetospeed(&serial_options, B115200);  // 设置输出波特率

    // 配置串口参数
    serial_options.c_cflag |= (CLOCAL | CREAD);  // 使能串口，允许接收数据
    serial_options.c_cflag &= ~CSIZE;  // 清除字符大小标志
    serial_options.c_cflag |= CS8;  // 8位数据位
    serial_options.c_cflag &= ~CSTOPB;  // 1个停止位
    serial_options.c_cflag &= ~PARENB;  // 无校验位
    serial_options.c_cflag &= ~CRTSCTS;  // 无硬件流控制

    // 设置串口配置
    if (tcsetattr(serial_fd, TCSANOW, &serial_options) != 0)  // 设置串口参数
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set serial port attributes");
        close(serial_fd);
        throw std::runtime_error("Failed to set serial port attributes");
    }

    RCLCPP_INFO(this->get_logger(), "Serial port %s configured successfully", usart_port_name.c_str());
}

/**************************************
Date: May 31, 2020
Function: 写数据到串口
***************************************/
void TurnOnRobot::write_to_serial_port(uint8_t *data, size_t size)
{
    ssize_t bytes_written = write(serial_fd, data, size);
    // if (bytes_written < 0)
    // {
    //     throw std::runtime_error("Failed to write to serial port");
    // }
}

/**************************************
Date: May 31, 2020
Function: 从串口读取数据
***************************************/
ssize_t TurnOnRobot::read_from_serial_port(uint8_t *buffer, size_t size)
{
    return read(serial_fd, buffer, size);  // 从串口读取数据
}

/**************************************
Date: May 31, 2020
Function: 关闭串口
***************************************/
void TurnOnRobot::close_serial_port()
{
    close(serial_fd);  // 关闭串口
}

/**************************************
Date: June 29, 2020
Function: 发布IMU数据
***************************************/
void TurnOnRobot::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub;  // 话题的消息类型 sensor_msgs::Imu
    Imu_Data_Pub.header.stamp = rclcpp::Clock().now();  // 当前时间
    Imu_Data_Pub.header.frame_id = "gyro_link";  
    Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;  // 四元数
    Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
    Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
    Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
    Imu_Data_Pub.orientation_covariance[0] = 1e6;
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;  // 三轴角速度
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;  // 三轴线性加速度
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
    imu_publisher->publish(Imu_Data_Pub);  // 发布 IMU 数据
}

/**************************************
Date: May 31, 2020
Function: 发布里程计相关信息
***************************************/
void TurnOnRobot::Publish_Odom()
{
    //geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, Robot_Pos.Z));
    // 使用 tf2::Quaternion 构造四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z);  // 设置 roll, pitch, yaw（Z轴旋转）

    // 手动提取四元数并赋值给 geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.x = q.x();
    odom_quat.y = q.y();
    odom_quat.z = q.z();
    odom_quat.w = q.w();

    nav_msgs::msg::Odometry odom;  // 里程计话题消息数据类型
    odom.header.stamp = rclcpp::Clock().now();  // 当前时间
    odom.header.frame_id = odom_frame_id;
    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z; 
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = robot_frame_id;
    odom.twist.twist.linear.x = Robot_Vel.X;
    odom.twist.twist.linear.y = Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z;
    odom_publisher->publish(odom);  // 发布里程计数据
}

/**************************************
Date: May 31, 2020
Function: 订阅回调函数
***************************************/
void TurnOnRobot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    short transition;  // 中间变量
    Send_Data2.tx[0] = FRAME_HEADER;  // 帧头 固定值
    Send_Data2.tx[1] = 1;  // 产品型号
    Send_Data2.tx[2] = 0;  // 机器人使能控制标志位
    // 机器人x轴的目标线速度
    transition = twist_aux->linear.x * 1000;  // 将浮点数放大一千倍，简化传输
    Send_Data2.tx[4] = transition;
    Send_Data2.tx[3] = transition >> 8;
    // 机器人y轴的目标线速度
    transition = twist_aux->linear.y * 1000;
    Send_Data2.tx[6] = transition;
    Send_Data2.tx[5] = transition >> 8;
    // 机器人z轴的目标角速度
    transition = twist_aux->angular.z * 1000;
    Send_Data2.tx[8] = transition;
    Send_Data2.tx[7] = transition >> 8;

    Send_Data2.tx[9] = Check_Sum(9, SEND_DATA_CHECK);  // 帧尾校验位，规则参见 Check_Sum 函数
    Send_Data2.tx[10] = FRAME_TAIL;  // 数据的最后一位是帧尾（固定值）
    try
    {
        write_to_serial_port(Send_Data2.tx, sizeof(Send_Data2.tx));  // 向串口发送数据
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Unable to send data through serial port");
    }
}
/**************************************
Date: May 31, 2020
Function: 订阅回调函数，根据订阅的指令向串口发指令控制下位机
***************************************/
void TurnOnRobot::joint_states_Callback(const sensor_msgs::msg::JointState::SharedPtr arm_joint)
{
    short transition;  // 中间变量
    Send_Data.tx[0] = FRAME_HEADER_ARM;  // 帧头 固定值

    // 遍历所有关节，将数据转换为字节并填充到 `Send_Data.tx` 数组中
    for (int i = 0; i < 6; ++i)
    {
        transition = arm_joint->position[i] * 1000;  // 将浮点数放大一千倍，简化传输
        Send_Data.tx[2 + 2 * i] = transition;  // 低字节
        Send_Data.tx[1 + 2 * i] = transition >> 8;  // 高字节
    }

    Send_Data.tx[13] = DEFAULT_MODE;
    Send_Data.tx[14] = Check_Sum(14, SEND_DATA_CHECK);  // 帧尾校验位
    Send_Data.tx[15] = FRAME_TAIL_ARM;  // 数据的最后一位是帧尾（固定值）

    try
    {
        write_to_serial_port(Send_Data.tx, sizeof(Send_Data.tx));  // 向串口发送数据
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Unable to send data through serial port");
    }
}

void TurnOnRobot::arm_teleop_Callback(const sensor_msgs::msg::JointState::SharedPtr arm_joint)
{
    short transition;  // 中间变量
    Send_Data.tx[0] = FRAME_HEADER_ARM;  // 帧头 固定值

    // 遍历所有关节，将数据转换为字节并填充到 `Send_Data.tx` 数组中
    for (int i = 0; i < 6; ++i)
    {
        transition = arm_joint->position[i] * 1000;  // 将浮点数放大一千倍，简化传输
        Send_Data.tx[2 + 2 * i] = transition;  // 低字节
        Send_Data.tx[1 + 2 * i] = transition >> 8;  // 高字节
    }

    Send_Data.tx[13] = DEFAULT_MODE;
    Send_Data.tx[14] = Check_Sum(14, SEND_DATA_CHECK);  // 帧尾校验位
    Send_Data.tx[15] = FRAME_TAIL_ARM;  // 数据的最后一位是帧尾（固定值）

    try
    {
        write_to_serial_port(Send_Data.tx, sizeof(Send_Data.tx));  // 向串口发送数据
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Unable to send data through serial port");
    }
}

/**************************************
Date: May 31, 2020
Function: 发布电压相关信息
***************************************/
void TurnOnRobot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;  // 定义电源电压发布话题的数据类型 std_msgs::Float32
    static float Count_Voltage_Pub = 0;
    if (Count_Voltage_Pub++ > 10)
    {
        Count_Voltage_Pub = 0;
        voltage_msgs.data = Power_voltage;  // 电源供电的电压获取
        voltage_publisher->publish(voltage_msgs);  // 发布电源电压话题单位V
    }
}

/**************************************
Date: May 31, 2020
Function: 发布位置信息
***************************************/
void TurnOnRobot::Publish_Pose()
{
    //geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, Robot_Pos.Z));

    // 使用 tf2::Quaternion 构造四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z);  // 设置 roll, pitch, yaw（Z轴旋转）

    // 手动提取四元数并赋值给 geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.x = q.x();
    odom_quat.y = q.y();
    odom_quat.z = q.z();
    odom_quat.w = q.w();

    geometry_msgs::msg::Pose pose;  // 位置话题消息数据类型

    pose.position.x = Robot_Pos.X;  // 机器人位置
    pose.position.y = Robot_Pos.Y;
    pose.position.z = Robot_Pos.Z;
    pose.orientation = odom_quat;  // 机器人朝向

    pose_publisher->publish(pose);  // 发布位置信息
}

/**************************************
Date: May 31, 2020
Function: 控制循环，采集数据并发布信息
***************************************/
void TurnOnRobot::Control()
{
    //rclcpp::Rate rate(10);  // 控制循环频率为 10 Hz
    while (rclcpp::ok())
    {
        if (Get_Sensor_Data())  // 从串口读取下位机数据
        {
            Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;  // 计算 X 方向的位移
            Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;  // 计算 Y 方向的位移
            Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;  // 角位移

            Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);  // 四元数解算
            Publish_Odom();  // 发布里程计话题
            Publish_ImuSensor();  // 发布 IMU 数据
            Publish_Voltage();  // 发布电源电压数据
        }
        rclcpp::spin_some(shared_from_this());  // 调用回调函数处理
        //rate.sleep();  // 控制循环频率
    }
}

/**************************************
Date: May 31, 2020
Function: 从串口读取数据
***************************************/
bool TurnOnRobot::Get_Sensor_Data()
{
    short transition_16 = 0;
    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0};
    ssize_t bytes_read = read_from_serial_port(Receive_Data_Pr, sizeof(Receive_Data_Pr));  // 从串口读取数据

    if (bytes_read <= 0)
    {
        return false;
    }

    // 处理接收到的数据
    int Header_Pos = -1, Tail_Pos = -1;
    for (int j = 0; j < 24; ++j)
    {
        if (Receive_Data_Pr[j] == FRAME_HEADER)
            Header_Pos = j;
        else if (Receive_Data_Pr[j] == FRAME_TAIL)
            Tail_Pos = j;
    }

    if (Tail_Pos == (Header_Pos + 23))
    {
        memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else if (Header_Pos == (1 + Tail_Pos))
    {
        for (int j = 0; j < 24; ++j)
        {
            Receive_Data.rx[j] = Receive_Data_Pr[(j + Header_Pos) % 24];
        }
    }
    else
    {
        return false;
    }

    // 校验数据包的头和尾是否正确
    Receive_Data.Frame_Header = Receive_Data.rx[0];
    Receive_Data.Frame_Tail = Receive_Data.rx[23];

    if (Receive_Data.Frame_Header == FRAME_HEADER && Receive_Data.Frame_Tail == FRAME_TAIL)
    {
        if (Receive_Data.rx[22] == Check_Sum(22, READ_DATA_CHECK))  // 校验位检测
        {
            Receive_Data.Flag_Stop = Receive_Data.rx[1];  // 停止标志位
            Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);  // 获取底盘 X 方向速度
            Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);  // 获取底盘 Y 方向速度
            Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);  // 获取底盘 Z 方向速度

            Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);  // 获取 IMU 的 X 轴加速度
            Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);  // 获取 IMU 的 Y 轴加速度
            Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);  // 获取 IMU 的 Z 轴加速度

            Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);  // 获取 IMU 的 X 轴角速度
            Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);  // 获取 IMU 的 Y 轴角速度
            Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);  // 获取 IMU 的 Z 轴角速度

            // 线性加速度和角速度单位转换
            Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEL_RATIO;
            Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEL_RATIO;
            Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEL_RATIO;

            Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
            Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
            Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

            // 获取电池电压
            transition_16 = 0;
            transition_16 |= Receive_Data.rx[20] << 8;
            transition_16 |= Receive_Data.rx[21];
            Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001;  // 转换为实际电压

            return true;
        }
    }
    return false;
}

// Check_Sum 实现
unsigned char TurnOnRobot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
    unsigned char check_sum = 0, k;

    if (mode == 0)  // 接收数据
    {
        for (k = 0; k < Count_Number; k++)  // Count_Number 是接收数组位数减1
        {
            check_sum = check_sum ^ Receive_Data.rx[k];  // 按位异或
        }
    }

    if (mode == 1)  // 发送数据
    {
        // 机械臂校验位
        if (Count_Number == 14)
        {
            for (k = 0; k < Count_Number; k++)  // Count_Number 是发送数组位数减1
            {
                check_sum = check_sum ^ Send_Data.tx[k];  // 按位异或
            }
        }
        // 底盘校验位
        if (Count_Number == 9)
        {
            for (k = 0; k < Count_Number; k++)  // Count_Number 是发送数组位数减1
            {
                check_sum = check_sum ^ Send_Data2.tx[k];  // 按位异或
            }
        }
    }
    return check_sum;  // 返回结果
}

// Odom_Trans 实现
float TurnOnRobot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
    short transition_16;
    float data_return;
    transition_16 = 0;
    transition_16 |= Data_High << 8;  // 获取数据的高8位
    transition_16 |= Data_Low;       // 获取数据的低8位
    data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001;  // 单位转换
    return data_return;
}

// IMU_Trans 实现
short TurnOnRobot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High << 8;  // 获取数据的高8位
    transition_16 |= Data_Low;       // 获取数据的低8位
    return transition_16;
}


void TurnOnRobot::init_joint_states()
{
    RCLCPP_INFO(this->get_logger(), "arm is ready");  // 使用 ROS2 的日志打印
    short transition = 0;  // 中间变量

    // 设置帧头
    Send_Data.tx[0] = FRAME_HEADER_ARM;

    // 初始化机器人各关节的位置数据
    transition = 0 * 1000;  // 将浮点数放大一千倍，简化传输
    Send_Data.tx[2] = transition;   // 低8位
    Send_Data.tx[1] = transition >> 8;  // 高8位

    transition = 0 * 1000;
    Send_Data.tx[4] = transition;
    Send_Data.tx[3] = transition >> 8;

    transition = 0 * 1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition >> 8;

    transition = 1.57 * 1000;  // 设置一个初始值，1.57 是机械臂的默认角度（例如 pi/2）
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition >> 8;

    transition = 0 * 1000;
    Send_Data.tx[10] = transition;
    Send_Data.tx[9] = transition >> 8;

    transition = 0 * 1000;
    Send_Data.tx[12] = transition;
    Send_Data.tx[11] = transition >> 8;

    Send_Data.tx[13] = DEFAULT_MODE;  // 设置模式

    // 校验和：将数据进行校验
    Send_Data.tx[14] = Check_Sum(14, SEND_DATA_CHECK);
    Send_Data.tx[15] = FRAME_TAIL_ARM;  // 设置帧尾

    // 尝试通过串口发送数据
    try
    {
        // 使用串口发送数据
        //Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));  // 发送数据
        write_to_serial_port(Send_Data.tx, sizeof(Send_Data.tx));  // 向串口发送数据
        RCLCPP_INFO(this->get_logger(), "New control command sent");  // ROS2 中打印信息
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port: %s", e.what());
    }
}