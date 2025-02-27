/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Software License Agreement (BSD License 2.0)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of {copyright_holder} nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Darby Lim
 */

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"

#include <cstdio>  // For getchar, putchar
#include <cstdlib> // For system
#include <map>     // For std::map
#include <string>  // For std::string

#ifdef _WIN32
#include <conio.h> // For _kbhit, _getch (Windows-specific)
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

using namespace std::chrono_literals;

const double BURGER_MAX_LIN_VEL = 0.22;
const double BURGER_MAX_ANG_VEL = 2.84;
const double WAFFLE_MAX_LIN_VEL = 0.26;
const double WAFFLE_MAX_ANG_VEL = 1.82;
const double LIN_VEL_STEP_SIZE = 0.01;
const double ANG_VEL_STEP_SIZE = 0.1;

const char *msg = R"(
 Control Your carrrrrrrrrr!
 ---------------------------
 Moving around:
    u    i    o
    j    k    l
    m    ,    .
 
 q/z : increase/decrease max speeds by 10%
 w/x : increase/decrease only linear speed by 10%
 e/c : increase/decrease only angular speed by 10%
 space key, k : force stop
 anything else : stop smoothly
 b : switch to OmniMode/CommonMode
 ctrl-C to quit
 )";

const char *e = R"(
 Communications Failed
 )";

class WheeltecKeyboard : public rclcpp::Node
{
public:
    WheeltecKeyboard() : Node("wheeltec_keyboard"), speed_(0.2), turn_(1.0), x_(0.0), th_(0.0), count_(0),
                         target_speed_(0.0), target_turn_(0.0), target_HorizonMove_(0.0), control_speed_(0.0),
                         control_turn_(0.0), control_HorizonMove_(0.0), Omni_(false)
    {

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // 新增 shutdown 发布器
        shutdown_pub_ = this->create_publisher<std_msgs::msg::Bool>("/shutdown_signal", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&WheeltecKeyboard::timer_callback, this)); // 100Hz

        // Initialize moveBindings and speedBindings
        moveBindings_['i'] = {1, 0};
        moveBindings_['o'] = {1, -1};
        moveBindings_['j'] = {0, 1};
        moveBindings_['l'] = {0, -1};
        moveBindings_['u'] = {1, 1};
        moveBindings_[','] = {-1, 0};
        moveBindings_['.'] = {-1, -1};
        moveBindings_['m'] = {-1, 1};

        speedBindings_['q'] = {1.1, 1.1};
        speedBindings_['z'] = {0.9, 0.9};
        speedBindings_['w'] = {1.1, 1};
        speedBindings_['x'] = {0.9, 1};
        speedBindings_['e'] = {1, 1.1};
        speedBindings_['c'] = {1, 0.9};

#ifndef _WIN32 // Non-Windows systems (Linux, macOS)
        // Get terminal settings
        tcgetattr(STDIN_FILENO, &settings_);
        original_settings_ = settings_;

        // Set terminal to raw mode (non-blocking, no echo)
        settings_.c_lflag &= ~(ICANON | ECHO);        // Disable canonical mode and echo
        settings_.c_cc[VMIN] = 0;                     // Read doesn't block
        settings_.c_cc[VTIME] = 0;                    // No timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &settings_); // Apply immediately.

#endif
        print_vels();
        RCLCPP_INFO(this->get_logger(), "%s", msg); // print msg
    }

    ~WheeltecKeyboard()
    {
#ifndef _WIN32
        tcsetattr(STDIN_FILENO, TCSANOW, &original_settings_);
#endif
    }

    void timer_callback()
    {
        char key = get_key();

        if (key == 'b')
        {
            Omni_ = !Omni_;
            if (Omni_)
            {
                RCLCPP_INFO(this->get_logger(), "Switch to OmniMode");
                moveBindings_['.'] = {-1, -1};
                moveBindings_['m'] = {-1, 1};
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Switch to CommonMode");
                moveBindings_['.'] = {-1, 1};  // Corrected mode switch
                moveBindings_['m'] = {-1, -1}; // Corrected mode switch
            }
        }
        else if (moveBindings_.count(key))
        {
            x_ = moveBindings_[key][0];
            th_ = moveBindings_[key][1];
            count_ = 0;
        }
        else if (speedBindings_.count(key))
        {
            speed_ = speed_ * speedBindings_[key][0];
            turn_ = turn_ * speedBindings_[key][1];
            count_ = 0;
            print_vels();
        }
        else if (key == ' ' || key == 'k')
        {
            x_ = 0;
            th_ = 0.0;
            control_speed_ = 0.0;
            control_turn_ = 0.0;
            control_HorizonMove_ = 0.0;
        }
        else
        {
            count_++;
            if (count_ > 4)
            {
                x_ = 0;
                th_ = 0.0;
            }
        }

        target_speed_ = speed_ * x_;
        target_turn_ = turn_ * th_;
        target_HorizonMove_ = speed_ * th_;

        //  // Smooth control (acceleration/deceleration)
        //  control_speed_ = smooth_control(target_speed_, control_speed_, 0.1);
        //  control_turn_ = smooth_control(target_turn_, control_turn_, 0.5);
        //  control_HorizonMove_ = smooth_control(target_HorizonMove_, control_HorizonMove_, 0.1);
        control_speed_ = target_speed_;
        control_turn_ = target_turn_;
        control_HorizonMove_ = target_HorizonMove_;

        geometry_msgs::msg::Twist twist;
        if (!Omni_)
        {
            twist.linear.x = control_speed_;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = control_turn_;
        }
        else
        {
            twist.linear.x = control_speed_;
            twist.linear.y = control_HorizonMove_;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
        }
        publisher_->publish(twist);
    }

private:
    double smooth_control(double target, double current, double step)
    {
        if (target > current)
        {
            return std::min(target, current + step);
        }
        else if (target < current)
        {
            return std::max(target, current - step);
        }
        else
        {
            return target;
        }
    }

    char get_key()
    {
        char key = 0;
#ifdef _WIN32
        if (_kbhit())
        { // Check if a key has been pressed
            key = _getch();
        }
#else // Non-Windows systems

        // Use select() for non-blocking keyboard input
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0; // No wait

        // Check if the input is ready.
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0)
        {
            read(STDIN_FILENO, &key, 1);
        }

#endif
        return key;
    }

    void print_vels()
    {
        RCLCPP_INFO(this->get_logger(), "currently:\tspeed %f\tturn %f ", speed_, turn_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double speed_;
    double turn_;
    double x_;
    double th_;
    int count_;
    double target_speed_;
    double target_turn_;
    double target_HorizonMove_;
    double control_speed_;
    double control_turn_;
    double control_HorizonMove_;
    bool Omni_; // Use bool instead of int for clarity

    std::map<char, std::vector<int>> moveBindings_;
    std::map<char, std::vector<double>> speedBindings_;

#ifndef _WIN32
    struct termios settings_;
    struct termios original_settings_;
#endif
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheeltecKeyboard>();
    rclcpp::spin(node); // Keep running.
    rclcpp::shutdown();
    return 0;
}