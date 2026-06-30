#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>
#include <string>
#include <signal.h>

#define STOP 0
#define STRAIGHT 1
#define TURNLEFT 2
#define TURNRIGHT 3
#define ARCLEFT 4
#define ARCRIGHT 5
#define BACKMOVE 6

#define NORMAL_CONTROL 0
#define SLAM_CONTROL 1

using namespace std::chrono_literals;

class SpeedPublisher : public rclcpp::Node
{
public:
    SpeedPublisher()
    : Node("speed_publisher"),
      v_w(0.0),
      v_x(0.0),
      hz(20.0),
      target_w(0.0),
      target_x(0.0),
      control_data(-1),
      dx(0.0),
      dw(0.0),
      diff_count(0),
      slam_x_speed(0.0),
      slam_w_speed(0.0),
      normal_x_speed(0.0),
      normal_w_speed(0.0),
      slow_x_speed(0.0),
      slow_w_speed(0.0),
      control_mode(0)
    {
        setup();
        param_load();
    }

    void setup()
    {
        //key_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("motor/control/speed", 10);
        key_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("filtered_key_vel", 10);
        keyboard_control_sub = this->create_subscription<std_msgs::msg::Int32>(
            "keyboard_control", 10,
            std::bind(&SpeedPublisher::keyboardControlCb, this, std::placeholders::_1));
        keyboard_control_mode_sub = this->create_subscription<std_msgs::msg::Int32>(
            "keyboard_control_mode", 10,
            std::bind(&SpeedPublisher::keyboardControlModeCb, this, std::placeholders::_1));

        key_vel_msg.linear.x = 0.0;
        key_vel_msg.linear.y = 0.0;
        key_vel_msg.linear.z = 0.0;
        key_vel_msg.angular.x = 0.0;
        key_vel_msg.angular.y = 0.0;
        key_vel_msg.angular.z = 0.0;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0/hz)),
            std::bind(&SpeedPublisher::timer_callback, this));
    }

    void param_load()
    {
        this->declare_parameter("speed_setting.slam_x_speed", 0.3);
        this->declare_parameter("speed_setting.slam_w_speed", 0.4);
        this->declare_parameter("speed_setting.normal_x_speed", 0.5);
        this->declare_parameter("speed_setting.normal_w_speed", 0.8);
        this->declare_parameter("speed_setting.slow_x_speed", 0.2);
        this->declare_parameter("speed_setting.slow_w_speed", 0.4);

        slam_x_speed = this->get_parameter("speed_setting.slam_x_speed").as_double();
        slam_w_speed = this->get_parameter("speed_setting.slam_w_speed").as_double();
        normal_x_speed = this->get_parameter("speed_setting.normal_x_speed").as_double();
        normal_w_speed = this->get_parameter("speed_setting.normal_w_speed").as_double();
        slow_x_speed = this->get_parameter("speed_setting.slow_x_speed").as_double();
        slow_w_speed = this->get_parameter("speed_setting.slow_w_speed").as_double();
    }

private:
    void timer_callback()
    {
        controlLoop(control_data);
        keyVelPub();
    }

    void keyVelPub()
    {
        key_vel_msg.linear.x = v_x;
        key_vel_msg.angular.z = v_w;
        key_vel_pub->publish(key_vel_msg);
    }

    void controlLoop(int a)
    {
        if(a != -1)
        {
            switch(a)
            {
                case 0:
                    target_x = 0.0;
                    target_w = 0.0;
                    break;
                case 1:
                    if(control_mode != 1)
                    {
                        target_x = normal_x_speed;
                        target_w = 0.0;
                    }
                    else
                    {
                        target_x = slam_x_speed;
                        target_w = 0.0;
                    }
                    break;
                case 2:
                    if(control_mode != 1)
                    {
                        target_x = 0.0;
                        target_w = normal_w_speed;
                    }
                    else
                    {
                        target_x = 0.0;
                        target_w = slam_w_speed;
                    }
                    break;
                case 3:
                    if(control_mode != 1)
                    {
                        target_x = 0.0;
                        target_w = -normal_w_speed;
                    }
                    else
                    {
                        target_x = 0.0;
                        target_w = -slam_w_speed;
                    }
                    break;
                case 4:
                    if(control_mode != 1)
                    {
                        target_x = normal_x_speed * 0.75;
                        target_w = normal_w_speed;
                    }
                    else
                    {
                        target_x = slam_x_speed * 0.75;
                        target_w = slam_w_speed;
                    }
                    break;
                case 5:
                    if(control_mode != 1)
                    {
                        target_x = normal_x_speed * 0.75;
                        target_w = -normal_w_speed;
                    }
                    else
                    {
                        target_x = slam_x_speed * 0.75;
                        target_w = -slam_w_speed;
                    }
                    break;
                case 6:
                    if(control_mode != 1)
                    {
                        target_x = -normal_x_speed/2.0;
                        target_w = 0.0;
                    }
                    else
                    {
                        target_x = -slam_x_speed/2.0;
                        target_w = 0.0;
                    }
                    break;
                case 7:
                    target_x = slow_x_speed * 0.75;
                    target_w = slow_w_speed;
                    break;
                case 8:
                    target_x = slow_x_speed * 0.75;
                    target_w = -slow_w_speed;
                    break;
                case 9:
                    target_x = slow_x_speed;
                    target_w = 0.0;
                    break;
                case 10:
                    target_x = 0.0;
                    target_w = slow_w_speed;
                    break;
                case 11:
                    target_x = 0.0;
                    target_w = -slow_w_speed;
                    break;
            }
            diff_count = 20;
            dx = (target_x - v_x)/diff_count;
            dw = (target_w - v_w)/diff_count;
            
            control_data = -1;
        }

        if(diff_count != 0)
        {
            v_x += dx;
            v_w += dw;
            diff_count--;
        }

        if(std::abs(v_x) < 0.000001)
        {
            v_x = 0.0;
        }

        if(std::abs(v_w) < 0.000001)
        {
            v_w = 0.0;
        }
    }

    void keyboardControlCb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        control_data = msg->data;
    }

    void keyboardControlModeCb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        control_mode = msg->data;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr key_vel_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr keyboard_control_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr keyboard_control_mode_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist key_vel_msg;
    double v_w, v_x;
    double target_w, target_x;
    double hz;
    double dx, dw;
    int control_data;
    int diff_count;
    double slam_x_speed, slam_w_speed, normal_x_speed, normal_w_speed, slow_x_speed, slow_w_speed;
    int control_mode;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}