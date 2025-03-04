#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>

// 声明全局变量以便外部信号处理
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor;
bool g_shutdown_requested = false;

// 信号处理函数
void signalHandler(int signum) {
    std::cout << "接收到信号 (" << signum << ")，准备关闭..." << std::endl;
    g_shutdown_requested = true;
    if (g_executor) {
        g_executor->cancel();
    }
}

class RobotLauncher : public rclcpp::Node {
public:
    RobotLauncher() : Node("robot_launcher") {
        // 设置一个定时器用于监控关闭请求
        shutdown_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { this->check_shutdown(); });
            
        RCLCPP_INFO(this->get_logger(), "机器人启动节点初始化完成");
    }

private:
    void check_shutdown() {
        if (g_shutdown_requested) {
            RCLCPP_INFO(this->get_logger(), "开始关闭程序...");
            rclcpp::shutdown();
        }
    }

    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char** argv) {
    // 注册信号处理函数
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    
    // 创建多线程执行器
    g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    std::cout << "正在启动机器人驱动和键盘控制..." << std::endl;
    
    // 创建启动器节点
    auto launcher_node = std::make_shared<RobotLauncher>();
    g_executor->add_node(launcher_node);
    
    // 使用ROS 2的方式启动其他节点
    // 注意：这些命令会在单独的终端窗口中运行，当按Ctrl+C关闭主程序时，这些窗口会保持打开
    // 因为它们是独立的进程
    std::string cmd1 = "gnome-terminal --title='机器人驱动' -- bash -c 'ros2 run turn_on_robot_driver turn_on_robot_driver; read -p \"按回车键关闭\"'";
    std::string cmd2 = "gnome-terminal --title='键盘控制' -- bash -c 'sleep 2 && ros2 run turn_on_robot_driver keycontrol; read -p \"按回车键关闭\"'";
    
    // 在分离的线程中启动这些命令，以避免阻塞主线程
    std::thread driver_thread([cmd1]() {
        int ret = std::system(cmd1.c_str());
        if (ret != 0) {
            std::cerr << "注意: 机器人驱动窗口可能无法启动，返回码: " << ret << std::endl;
        }
    });
    driver_thread.detach();  // 分离线程
    
    std::thread keycontrol_thread([cmd2]() {
        int ret = std::system(cmd2.c_str());
        if (ret != 0) {
            std::cerr << "注意: 键盘控制窗口可能无法启动，返回码: " << ret << std::endl;
        }
    });
    keycontrol_thread.detach();  // 分离线程
    
    std::cout << "所有节点已启动。按Ctrl+C退出启动器。" << std::endl;
    std::cout << "注意：机器人驱动和键盘控制在独立窗口中运行，需要单独关闭。" << std::endl;
    
    // 启动执行器
    g_executor->spin();
    
    // 清理资源
    rclcpp::shutdown();
    std::cout << "启动器已关闭。" << std::endl;
    
    return 0;
}
