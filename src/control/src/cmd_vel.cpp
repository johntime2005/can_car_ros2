#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "control/canbus.h"

int dev = 0;
int cpot0 = 0;
int cpot1 = 1;
Can_Msg txmsg1[100];
Can_Msg txmsg2[100];
Can_Msg txmsg3[100];
Can_Msg rxmsg[100];



//打印报文和ID
void print_frame(Can_Msg* msg) {
    printf("Sending CAN Frame:\n");
    printf("ID: 0x%X, Data: ", msg->ID);
    for (int i = 0; i < msg->DataLen; i++) {
        printf("%02X ", msg->Data[i]);
    }
    printf("\n");
}

class CanBusControlNode : public rclcpp::Node {
public:
    CanBusControlNode() : Node("canbus_control"), frame_counter(0) {
        // 初始化CAN设备及配置
        int devs, ret;
        Can_Config cancfg;
        devs = CAN_ScanDevice();
        if(devs <= 0) {
            RCLCPP_ERROR(this->get_logger(), "No CAN device found");
            rclcpp::shutdown();
            return;
        }
        ret = CAN_OpenDevice(dev, cpot0);
        if(ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN_OpenDevice channel0 failed");
            rclcpp::shutdown();
            return;
        }
        ret = CAN_OpenDevice(dev, cpot1);
        if(ret != 0) {
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
        if(ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN_Init channel0 failed");
            rclcpp::shutdown();
            return;
        }
        CAN_SetFilter(dev, cpot0, 0, 0, 0, 0, 1);
        ret = CAN_Init(dev, cpot1, &cancfg);
        if(ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN_Init channel1 failed");
            rclcpp::shutdown();
            return;
        }
        CAN_SetFilter(dev, cpot1, 0, 0, 0, 0, 1);
        
        // 创建定时器，每10毫秒回调一次
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&CanBusControlNode::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "CANBusControl node initialized");
    }

private:
    void timerCallback() {
        switch (frame_counter) {
            case 0:
                send_frame1();
                RCLCPP_INFO(this->get_logger(), "send_frame1 = 0x%x", txmsg1[0].ID);
                break;
            case 1:
                send_frame2();
                RCLCPP_INFO(this->get_logger(), "send_frame2 = 0x%x", txmsg2[0].ID);
                break;
            case 2:
                send_frame3();
                RCLCPP_INFO(this->get_logger(), "send_frame3 = 0x%x", txmsg3[0].ID);
                break;
        }
        frame_counter = (frame_counter + 1) % 3;
    }

    // CAN帧发送函数，直接沿用原有实现
    void send_frame1() {
        memset(&txmsg1[0], 0, sizeof(txmsg1[0]));
        txmsg1[0].ID = 0x18C4D1D0;
        txmsg1[0].Data[0] = 0x83;
        txmsg1[0].Data[1] = 0x3E;
        txmsg1[0].Data[2] = 0x00;
        txmsg1[0].Data[3] = 0x00;
        txmsg1[0].Data[4] = 0x00;
        txmsg1[0].Data[5] = 0x00;
        txmsg1[0].Data[6] = 0x00;
        txmsg1[0].Data[7] = 0xBD;
        txmsg1[0].DataLen = 8;
        txmsg1[0].ExternFlag = 1;
        CAN_Transmit(dev, cpot0, &txmsg1[0], 1, 100);
        print_frame(&txmsg1[0]);
    }

    void send_frame2() {
        memset(&txmsg2[0], 0, sizeof(txmsg2[0]));
        txmsg2[0].ID = 0x18C4D1D0;
        txmsg2[0].Data[0] = 0x83;
        txmsg2[0].Data[1] = 0x3E;
        txmsg2[0].Data[2] = 0x00;
        txmsg2[0].Data[3] = 0x00;
        txmsg2[0].Data[4] = 0x00;
        txmsg2[0].Data[5] = 0x00;
        txmsg2[0].Data[6] = 0x10;
        txmsg2[0].Data[7] = 0xAD;
        txmsg2[0].DataLen = 8;
        txmsg2[0].ExternFlag = 1;
        CAN_Transmit(dev, cpot0, txmsg2, 1, 100);
        print_frame(&txmsg2[0]);
    }

    void send_frame3() {
        memset(&txmsg3[0], 0, sizeof(txmsg3[0]));
        txmsg3[0].ID = 0x18C4D1D0;
        txmsg3[0].Data[0] = 0x83;
        txmsg3[0].Data[1] = 0x00;
        txmsg3[0].Data[2] = 0xC0;
        txmsg3[0].Data[3] = 0x63;
        txmsg3[0].Data[4] = 0x0F;
        txmsg3[0].Data[5] = 0x00;
        txmsg3[0].Data[6] = 0x20;
        txmsg3[0].Data[7] = 0x8F;
        txmsg3[0].DataLen = 8;
        txmsg3[0].ExternFlag = 1;
        CAN_Transmit(dev, cpot0, txmsg3, 1, 100);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int frame_counter;
};

// main函数：ROS2节点初始化及spin
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanBusControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


