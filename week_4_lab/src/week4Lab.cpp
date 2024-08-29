#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <random>
#include <chrono>
#include <cmath>

class DeadReckoningNode : public rclcpp::Node {

    public:
        DeadReckoningNode() : Node("dead_reckoning_node"), gen_(rd_()), noise_dist_(0.0, 0.01) {
            linear_speed_ = this->declare_parameter("linear_speed", 0.2);
            angular_speed_ = this->declare_parameter("angular_speed", 0.0);
            distance_ = this->declare_parameter("distance", 2.0);
            direction_ = this->declare_parameter("direction", "forward");

            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            odom_noisy_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_noisy", 10);
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&DeadReckoningNode::odom_callback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&DeadReckoningNode::timer_callback, this));

            initialised_ = false;
            last_time_ = this->now();
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const 
        {

        }

        void timer_callback()
        {
            if(!initialised_){
                

            }
        }

        // double linear_speed_;
        // double angular_speed_;
        // double distance_;
        // std::string direction_;

        // bool initialised_;
        // auto last_time_;

        // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_noisy_pub_;
        // rclcpp::Subscriber<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
        
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoningNode>());
  rclcpp::shutdown();
  return 0;
}