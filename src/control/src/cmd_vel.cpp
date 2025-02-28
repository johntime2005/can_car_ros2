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
                          target_rad_(0),
                          current_linear_speed_(0.0),   // 新增：当前线速度
                          current_angular_speed_(0.0)   // 新增：当前角速度
    {
        // ... CAN 初始化代码 (与之前相同) ...
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

        // 创建定时器，每10ms执行一次回调函数
        timer_ = this->create_wall_timer(10ms, std::bind(&CanBusControlNode::timer_callback, this));
    }

    ~CanBusControlNode()
    {
        // ... 关闭 CAN 通道代码 (与之前相同) ...
        // 关闭CAN通道，释放设备资源 (假设CAN_CloseDevice存在)
        CAN_CloseDevice(dev, cpot0);
        CAN_CloseDevice(dev, cpot1);
        RCLCPP_INFO(this->get_logger(), "CAN channels closed.");
    }

    // 通过订阅更新目标参数 (与之前相同)
    void set_target(double linear, double angular)
    {
        target_speed_ = static_cast<int>(linear * 1000);
        target_rad_ = static_cast<int>(angular * 100);
    }

    // 获取当前线速度
    double getLinearSpeed() const { return current_linear_speed_; }

    // 获取当前角速度
    double getAngularSpeed() const { return current_angular_speed_; }

      // 计算校验和 (与之前版本相同)
    uint8_t calculateChecksum(const uint8_t data[], int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
    // 解析 ctrl_fb 报文，并更新速度变量 (修改)
    void parseCtrlFbMessage(const uint8_t data[], int len)
    {
        // ... (校验和检查、档位解析等代码与之前相同) ...
        if (len != 8)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: ctrl_fb message should have 8 bytes.");
            return;
        }

        // 校验和检查
        uint8_t receivedChecksum = data[7];
        uint8_t calculatedChecksum = calculateChecksum(data, len);
        if (receivedChecksum != calculatedChecksum)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Checksum mismatch. Received: 0x%02X, Calculated: 0x%02X",
                         receivedChecksum, calculatedChecksum);
            return;
        }

        // 解析档位
        uint8_t gear = data[0] & 0x0F;
        std::string gear_str;
        switch (gear)
        {
        case 0x00:
            gear_str = "disable";
            break;
        case 0x01:
            gear_str = "Parking";
            break;
        case 0x02:
            gear_str = "Neutral";
            break;
        case 0x03:
            gear_str = "Kinematic Control";
            break;
        case 0x04:
            gear_str = "Free Control";
            break;
        default:
            gear_str = "Unknown (" + std::to_string(gear) + ")";
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Gear: %s", gear_str.c_str());

        // 解析当前车体线速度
        int16_t linearSpeedRaw = (data[1] << 8) | (data[0] & 0xF0);
        linearSpeedRaw = linearSpeedRaw >> 4;
        current_linear_speed_ = static_cast<double>(linearSpeedRaw) * 0.001; // 更新成员变量
        RCLCPP_INFO(this->get_logger(), "Linear Speed: %.3f m/s", current_linear_speed_.load());

        // 解析当前车体角速度
        int16_t angularSpeedRaw = (data[3] << 8) | data[2];
        current_angular_speed_ = static_cast<double>(angularSpeedRaw) * 0.01; // 更新成员变量
        RCLCPP_INFO(this->get_logger(), "Angular Speed: %.2f deg/s", current_angular_speed_.load());

        // 解析Alive Rolling Counter
        uint8_t counter = data[6] & 0x0F;
        RCLCPP_INFO(this->get_logger(), "Alive Rolling Counter: %d", counter);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<int> target_speed_;
    std::atomic<int> target_rad_;
    std::atomic<double> current_linear_speed_;  // 新增：当前线速度
    std::atomic<double> current_angular_speed_;  // 新增：当前角速度

    // ... 其余辅助函数 (print_frame, calculate_speed, move) ...
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
        // 1. 发送控制指令 (根据 cmd_vel 设定的目标速度)
        move(target_speed_, target_rad_);

        // 2. 接收反馈数据
        int receivedCount = CAN_Receive(dev, cpot0, rxmsg, 100, 100);
        if (receivedCount > 0)
        {
            // 3. 处理接收到的数据
            for (int i = 0; i < receivedCount; i++)
            {
                if (rxmsg[i].ID == 0x18C4D1EF)
                { // 检查是否为 ctrl_fb 报文
                    parseCtrlFbMessage(rxmsg[i].Data, rxmsg[i].DataLen);

                    RCLCPP_INFO(this->get_logger(), "Current Speeds: Linear=%.3f m/s, Angular=%.2f deg/s",
                                current_linear_speed_.load(), current_angular_speed_.load());
                }
            }
        }
        else if (receivedCount < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: CAN_Receive failed.");
        }
    }
};

class CmdVelSubscriber : public rclcpp::Node
{
public:
     CmdVelSubscriber(std::shared_ptr<CanBusControlNode> canbus_node)
        : Node("cmd_vel_subscriber"), canbus_node_(canbus_node)
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
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

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(canbus_node);
    executor.add_node(twist_subscriber);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}