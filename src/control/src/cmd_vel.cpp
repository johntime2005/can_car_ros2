#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "control/canbus.h"
#include "std_msgs/msg/bool.hpp"
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

int dev = 0;
int cpot0 = 0;
int cpot1 = 1;
Can_Msg txmsg1[100];
Can_Msg txmsg2[100];
Can_Msg txmsg3[100];
Can_Msg rxmsg[100];

class CanBusControlNode : public rclcpp::Node
{
public:
    CanBusControlNode() : Node("canbus_control"),
                          target_speed_(0),
                          target_rad_(0)
    {
        // 初始化CAN设备及配置 (这部分代码基本保持不变)
        int devs, ret;
        Can_Config cancfg;
        devs = CAN_ScanDevice();
        if (devs <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "No CAN device found");
            rclcpp::shutdown();
            return;
        }
        ret = CAN_OpenDevice(dev, cpot0);
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN_OpenDevice channel0 failed");
            rclcpp::shutdown();
            return;
        }
        ret = CAN_OpenDevice(dev, cpot1);
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN_OpenDevice channel1 failed");
            rclcpp::shutdown();
            return;
        }
        // CAN配置
        cancfg.model = 0;
        cancfg.configs = 0;
        cancfg.baudrate = 500000;
        cancfg.configs |= 0x0001;
        cancfg.configs |= 0x0002;
        cancfg.configs |= 0x0004;
        ret = CAN_Init(dev, 0, &cancfg);
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN_Init channel0 failed");
            rclcpp::shutdown();
            return;
        }
        CAN_SetFilter(dev, cpot0, 0, 0, 0, 0, 1);
        ret = CAN_Init(dev, cpot1, &cancfg);
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAN_Init channel1 failed");
            rclcpp::shutdown();
            return;
        }
        CAN_SetFilter(dev, cpot1, 0, 0, 0, 0, 1);

        // 添加 shutdown 信号订阅
        shutdown_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/shutdown_signal", 10,
            std::bind(&CanBusControlNode::shutdown_callback, this, std::placeholders::_1));

        // 创建定时器，每10ms发送一次控制指令
        timer_ = this->create_wall_timer(10ms, std::bind(&CanBusControlNode::timer_callback, this));
    }

    ~CanBusControlNode()
    {
        // 关闭CAN通道，释放设备资源 (假设CAN_CloseDevice存在)
        CAN_CloseDevice(dev, cpot0);
        CAN_CloseDevice(dev, cpot1);
        RCLCPP_INFO(this->get_logger(), "CAN channels closed.");
    }

    // 通过订阅更新目标参数；输入的线速度单位 m/s，角速度单位为用户定义单位（如：°/s）
    void set_target(double linear, double angular)
    {
        // twist 消息的 linear.x 单位是 m/s，转换为 0.001 m/s 单位则要乘 1000
        target_speed_ = static_cast<int>(linear * 1000);
        // 同理，angular.z 单位转换乘以 100
        target_rad_ = static_cast<int>(angular * 100);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<int> target_speed_;
    std::atomic<int> target_rad_;

    // 新增 shutdown 话题回调
    void shutdown_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "Received shutdown signal, shutting down.");
            rclcpp::shutdown();
        }
    }

    void print_frame(Can_Msg *msg)
    {
        printf("Sending CAN Frame:\n");
        printf("ID: 0x%X, Data: ", msg->ID);
        for (int i = 0; i < msg->DataLen; i++)
        {
            printf("%02X ", msg->Data[i]);
        }
        printf("\n");
    }

    void calculate_speed(int speed, int rad, unsigned char *data)
    {
        // 目标速度 (单位: 0.001 m/s)
        short speed_short = static_cast<short>(speed);
        data[0] |= (speed_short & 0x0F) << 4;  // 低4位, 并保留目标档位
        data[1] |= (speed_short >> 4) & 0xFF;  // 高8位
        data[2] |= (speed_short >> 12) & 0x0F; // 最高4位

        // 目标角速度 (单位: 0.01 °/s)
        short rad_short = static_cast<short>(rad);
        data[2] |= (rad_short & 0x0F) << 4;  // 低4位
        data[3] |= (rad_short >> 4) & 0xFF;  // 高8位
        data[4] |= (rad_short >> 12) & 0x0F; // 最高4位
    }

    void move(int speed, int rad)
    {
        // 清零信号结构体
        memset(&txmsg1[0], 0, sizeof(txmsg1[0]));
        txmsg1[0].ID = 0x18C4D1D0;
        // 目标档位 (运动学控制模式)
        txmsg1[0].Data[0] = 0x03;

        // 填充速度、角速度信号
        calculate_speed(speed, rad, txmsg1[0].Data);
        txmsg1[0].Data[5] = 0;

        // 使用当前时间计算 Alive Rolling Counter（心跳信号）
        rclcpp::Time now = this->now();
        unsigned char heartbeat = static_cast<unsigned char>((now.nanoseconds() / 10000000ULL) % 16);
        txmsg1[0].Data[6] = heartbeat << 4;

        // Byte7: 校验和，计算方法为 Byte0～Byte6 的 XOR 值
        unsigned char checksum = 0;
        for (int i = 0; i < 7; i++)
        {
            checksum ^= txmsg1[0].Data[i];
        }
        txmsg1[0].Data[7] = checksum;

        txmsg1[0].DataLen = 8;
        txmsg1[0].ExternFlag = 1;

        // 发送CAN帧，超时时间设为100ms
        CAN_Transmit(dev, cpot0, &txmsg1[0], 1, 100);
        print_frame(&txmsg1[0]);
    }

    void timer_callback()
    {
        // 直接使用 target_speed_ 和 target_rad_
        int speed = target_speed_;
        int rad = target_rad_;
        move(speed, rad);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_sub_;
};

class CmdVelSubscriber : public rclcpp::Node
{
public:
    // 在构造函数中传入 CanBusControlNode 的指针
    CmdVelSubscriber(std::shared_ptr<CanBusControlNode> canbus_node)
        : Node("cmd_vel_subscriber"), canbus_node_(canbus_node)
    {
        // 在 CmdVelSubscriber 构造函数中明确使用绝对话题名
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, // 添加前导斜杠确保使用绝对话题名
            std::bind(&CmdVelSubscriber::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f, angular.z=%.2f",
                    msg->linear.x, msg->angular.z);
        // 将接收到的消息用于更新控制参数
        if (canbus_node_)
        {
            canbus_node_->set_target(msg->linear.x, msg->angular.z);
        }
    }
    std::shared_ptr<CanBusControlNode> canbus_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto canbus_node = std::make_shared<CanBusControlNode>();
    auto twist_subscriber = std::make_shared<CmdVelSubscriber>(canbus_node);

    // 使用多线程执行器同时处理 CAN 总线控制和 cmd_vel 订阅
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(canbus_node);
    executor.add_node(twist_subscriber);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}