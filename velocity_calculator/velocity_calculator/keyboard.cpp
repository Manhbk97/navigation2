#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <termios.h>
#include <signal.h>
#include <thread>

using namespace std::chrono_literals;

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl() : Node("keyboard_control")
    {
        setupTerminal();
        setup();
        RCLCPP_INFO(this->get_logger(), "Keyboard Control Node has been initialized");
    }

    ~KeyboardControl()
    {
        restoreTerminal();
        RCLCPP_INFO(this->get_logger(), "Keyboard Control Node is shutting down");
    }

    void setup()
    {
        // Publishers
        control_pub = this->create_publisher<std_msgs::msg::Int32>("keyboard_control", 10);
        control_mode_pub = this->create_publisher<std_msgs::msg::Int32>("keyboard_control_mode", 10);
        voice_pub = this->create_publisher<std_msgs::msg::Int32>("seromo_gui/sound", 10);
        face_change_pub = this->create_publisher<std_msgs::msg::Int32>("face_changer", 10);
        motor_power_pub = this->create_publisher<std_msgs::msg::Bool>("motor_power", 1);

        std::cout << msg << std::endl;
    }

    void run()
    {
        // 키보드 입력을 처리하는 스레드 시작
        std::thread keyboard_thread(&KeyboardControl::keyboardLoop, this);
        keyboard_thread.detach();  // 메인 스레드와 분리

        rclcpp::spin(shared_from_this());
    }

private:
    void keyboardLoop()
    {
        while (rclcpp::ok())
        {
            int c = getch();
            int a = -1;
            int mode = -1;
            int voice_quick_slot = -1;
            int face_change = -1;

            RCLCPP_DEBUG(this->get_logger(), "Received key: %d", c);

            if (c == 'k' || c == 'K')
                a = 0;
            else if (c == 'i')
                a = 1;
            else if (c == 'j')
                a = 2;
            else if (c == 'l')
                a = 3;
            else if (c == 'u')
                a = 4;
            else if (c == 'o')
                a = 5;
            else if (c == ',')
                a = 6;
            else if (c == 'U')
                a = 7;
            else if (c == 'O')
                a = 8;
            else if (c == 'I')
                a = 9;
            else if (c == 'J')
                a = 10;
            else if (c == 'L')
                a = 11;
            else if (c == 's' || c == 'S')
                mode = 1;
            else if (c == 'a' || c == 'A')
                mode = 0;
            else if (c == 'p' || c == 'P')
                motorPowerPublisher(false);
            else if (c == '\x03')  // Ctrl+C
            {
                RCLCPP_INFO(this->get_logger(), "Ctrl+C detected, initiating shutdown...");
                rclcpp::shutdown();
                break;
            }

            if (a != -1)
            {
                RCLCPP_DEBUG(this->get_logger(), "Publishing control command: %d", a);
                controlPublisher(a);
                motorPowerPublisher(true);
            }

            if (mode != -1)
            {
                RCLCPP_DEBUG(this->get_logger(), "Publishing control mode: %d", mode);
                controlModePublisher(mode);
            }

            if (voice_quick_slot != -1)
            {
                RCLCPP_DEBUG(this->get_logger(), "Publishing voice command: %d", voice_quick_slot);
                voicePublisher(voice_quick_slot);
            }

            if (face_change != -1)
            {
                RCLCPP_DEBUG(this->get_logger(), "Publishing face change: %d", face_change);
                faceChangerPublisher(face_change);
            }
        }
    }

    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);  // 캐노니컬 모드와 에코 비활성화
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

    void controlPublisher(int a)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = a;
        control_pub->publish(msg);
    }

    void motorPowerPublisher(bool a)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = a;
        motor_power_pub->publish(msg);
    }

    void controlModePublisher(int a)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = a;
        control_mode_pub->publish(msg);
    }

    void voicePublisher(int a)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = a;
        voice_pub->publish(msg);
    }

    void faceChangerPublisher(int a)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = a;
        face_change_pub->publish(msg);
    }

    int getch()
    {
        return getchar();
    }

    const char* msg = R"(
---------------------------
Reading from the keyboard and Publishing to control topics!
---------------------------
Moving around:
                u    i    o 
                j    k    l
                    ,     

Control modes:
  a : Normal mode
  s : SLAM mode
  p : Motor power off

CTRL-C to quit
   )";

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr voice_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr face_change_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_power_pub;

    // Terminal settings
    struct termios oldt_, newt_;
};

void mySigintHandler(int sig)
{
    (void)sig;
    RCLCPP_INFO(rclcpp::get_logger("keyboard_control"), "Received SIGINT signal, shutting down...");
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, mySigintHandler);
    
    auto node = std::make_shared<KeyboardControl>();
    
    try {
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("keyboard_control"), "Exception occurred: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}