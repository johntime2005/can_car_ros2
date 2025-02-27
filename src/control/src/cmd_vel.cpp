#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "control/canbus.h" // 假设你的 CAN 库头文件是 control/canbus.h
#include <string.h>         // 引入 memset
#include <stdio.h>

int dev = 0;
int cpot0 = 0;
int cpot1 = 1;
Can_Msg txmsg1[100];
Can_Msg txmsg2[100];
Can_Msg txmsg3[100];
Can_Msg rxmsg[100];

// 打印报文和ID
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

class CanBusControlNode : public rclcpp::Node
{
public:
    CanBusControlNode() : Node("canbus_control")
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

        // 创建定时器，每10ms发送一次控制指令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CanBusControlNode::timer_callback, this));
    }

private:
    void calculate_speed(int speed, int rad, unsigned char *data)
    {
        // 目标档位 (运动学控制模式)
        data[0] = 0x03;

        // 目标速度 (单位: 0.001 m/s)
        short speed_short = (short)(speed);   // 转换为short int类型
        data[0] |= (speed_short & 0x0F) << 4; // 低4位,并保留目标档位
        data[1] = (speed_short >> 4) & 0xFF;  // 高8位

        // 目标角速度 (单位: 0.01 °/s)
        short rad_short = (short)(rad);    // 转换为short int类型
        data[2] = (rad_short & 0x0F) << 4; // 低4位
        data[3] = (rad_short >> 4) & 0xFF; // 高8位
    }

    void move(int speed, int rad)
    {
        // 清零信号结构体
        memset(&txmsg1[0], 0, sizeof(txmsg1[0]));
        txmsg1[0].ID = 0x18C4D1D0;

        // 填充速度、角速度信号
        calculate_speed(speed, rad, txmsg1[0].Data);

        // Byte 5:  Byte5虽然手册没提，但是安全起见，我们还是设为0
        txmsg1[0].Data[5] = 0;

        // Byte 6: Alive Rolling Counter，每发送一帧递增, 取低4位
        static unsigned char counter = 0; // 加个static
        txmsg1[0].Data[6] = counter << 4; // 存入低四位
        counter = (counter + 1) & 0x0F;   // 保证计数值在 0~15 内循环

        // Byte7: 校验和，计算方法为 Byte0～Byte6 的 XOR 值
        unsigned char checksum = 0;
        for (int i = 0; i < 7; i++)
        {
            checksum ^= txmsg1[0].Data[i];
        }
        txmsg1[0].Data[7] = checksum;

        // 其他的一些设定
        txmsg1[0].DataLen = 8;
        txmsg1[0].ExternFlag = 1;

        // 发送CAN帧，超时时间设为100ms（具体值可调整）
        CAN_Transmit(dev, cpot0, &txmsg1[0], 1, 100);

        // 调用打印函数输出发送帧内容
        print_frame(&txmsg1[0]);
    }

    void timer_callback()
    {
        // 在这里设置你想要的速度和角速度
        int speed = 0;
        int rad = 30;
        speed = speed * 1000; // 转换为 0.001 m/s
        rad = rad * 100;      // 转换为 0.01 °/s

        // 调用 move 函数发送控制指令
        move(speed, rad);
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanBusControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}