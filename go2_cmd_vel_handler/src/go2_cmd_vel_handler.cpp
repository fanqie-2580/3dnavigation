#include <memory>

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/bool.hpp>

#include <unitree_api/msg/request.hpp>



// 包含官方提供的 SportClient 头文件

#include "common/ros2_sport_client.h" 



using namespace std::chrono_literals;



class Go2CmdVelBridge : public rclcpp::Node

{

public:

    // 修改构造函数初始化列表，将 this 指针传递给 sport_req_

    Go2CmdVelBridge() : Node("go2_cmd_vel_bridge"), sport_req_(this)

    {

        // 1. 初始化参数

        this->declare_parameter<double>("stop_timeout", 0.5);

        stop_timeout_ = this->get_parameter("stop_timeout").as_double();



        // 2. 订阅标准的 cmd_vel

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(

            "cmd_vel", 10, std::bind(&Go2CmdVelBridge::cmd_vel_callback, this, std::placeholders::_1));

        estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(

            "/web_estop", 10, std::bind(&Go2CmdVelBridge::estop_callback, this, std::placeholders::_1));



        // 3. 发布给 Unitree 运动控制 API (注意：SportClient 内部可能已经创建了发布者，但我们这里可以保留用于手动发包)
        estop_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/web_estop/state", 10);



        // 4. 定时器

        last_cmd_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(

            100ms, std::bind(&Go2CmdVelBridge::timer_callback, this));



        RCLCPP_INFO(this->get_logger(), "Go2 CmdVel Bridge Initialized with SportClient.");

        

        // Stand up, then enter BalanceStand so subsequent Move(vx,vy,vyaw) is accepted.
        send_recovery();
        balance_timer_ = this->create_wall_timer(
            1500ms, std::bind(&Go2CmdVelBridge::send_balance_once, this));

    }



private:

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)

    {

        if (estop_active_)

        {

            return;

        }

        last_cmd_time_ = this->get_clock()->now();

        current_vel_ = *msg;

        

        // 调用封装好的 Move 方法

        unitree_api::msg::Request req;

        sport_req_.Move(req, current_vel_.linear.x, current_vel_.linear.y, current_vel_.angular.z);
        

        is_running_ = true;

    }



    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg)

    {

        if (msg->data == estop_active_)

        {

            publish_estop_state();

            return;

        }

        estop_active_ = msg->data;

        publish_estop_state();

        if (estop_active_)

        {

            RCLCPP_WARN(this->get_logger(), "Web estop active. Stopping robot.");

            send_stop();

            is_running_ = false;

        }

        else

        {

            RCLCPP_WARN(this->get_logger(), "Web estop released.");

            last_cmd_time_ = this->get_clock()->now();

        }

    }



    void timer_callback()

    {

        if (estop_active_)

        {

            send_stop();

            return;

        }

        auto now = this->get_clock()->now();

        double diff = (now - last_cmd_time_).seconds();



        if (is_running_ && diff > stop_timeout_)

        {

            RCLCPP_WARN(this->get_logger(), "cmd_vel timeout! Stopping robot.");

            send_stop();

            is_running_ = false;

        }

    }



    void send_recovery()

    {

        RCLCPP_INFO(this->get_logger(), "Sending RecoveryStand command...");

        unitree_api::msg::Request req;

        // 修正方法名：根据报错提示，改为 RecoveryStand

        sport_req_.RecoveryStand(req);
    }

    void send_balance_once()
    {
        if (balance_timer_) {
            balance_timer_->cancel();
            balance_timer_.reset();
        }
        RCLCPP_INFO(this->get_logger(), "Sending BalanceStand command...");
        unitree_api::msg::Request req;
        sport_req_.BalanceStand(req);
    }



    void send_stop()

    {

        unitree_api::msg::Request req;

        sport_req_.StopMove(req);
    }




    void publish_estop_state()

    {

        std_msgs::msg::Bool state;

        state.data = estop_active_;

        estop_state_pub_->publish(state);

    }



    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr balance_timer_;



    // 关键修正：SportClient 将在构造函数中通过 this 初始化

    SportClient sport_req_;

    

    geometry_msgs::msg::Twist current_vel_;

    rclcpp::Time last_cmd_time_;

    double stop_timeout_;

    bool is_running_ = false;

    bool estop_active_ = false;

};



int main(int argc, char *argv[])

{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Go2CmdVelBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}