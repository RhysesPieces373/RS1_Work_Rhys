#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>

class SubscriberNode : public rclcpp::Node {

    public:
        SubscriberNode() : Node("odom_and_IMU") {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&SubscriberNode::odomCallback, this, std::placeholders::_1));
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&SubscriberNode::imuCallback, this, std::placeholders::_1));
            odom_avg_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom_avg", 10, std::bind(&SubscriberNode::odomAvgCallback, this, std::placeholders::_1));
            imu_avg_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu_avg", 10, std::bind(&SubscriberNode::imuAvgCallback, this, std::placeholders::_1));

            odom_avg_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_avg", 10);
            imu_avg_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_avg", 10);
            RCLCPP_INFO(this->get_logger(), "Listening for odom and IMU");
        }

    private:
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            // RCLCPP_INFO(this->get_logger(), "before pushed back odo");
            odom_vec_.push_back(*msg);
            // RCLCPP_INFO(this->get_logger(), "pushed back odo");
            if(odom_vec_.size() == 3){
                nav_msgs::msg::Odometry avg_msg;
                avg_msg.pose.pose.position.x = 0.0;
                avg_msg.pose.pose.position.y = 0.0;
                avg_msg.pose.pose.position.z = 0.0;
                avg_msg.pose.pose.orientation.x = 0.0;
                avg_msg.pose.pose.orientation.y = 0.0;
                avg_msg.pose.pose.orientation.z = 0.0;
                avg_msg.pose.pose.orientation.w = 0.0;

                for(int i = 0; i < 3; i++){
                    avg_msg.pose.pose.position.x += (odom_vec_.at(i)).pose.pose.position.x; 
                    avg_msg.pose.pose.position.y += (odom_vec_.at(i)).pose.pose.position.y; 
                    avg_msg.pose.pose.position.z += (odom_vec_.at(i)).pose.pose.position.z; 
                    avg_msg.pose.pose.orientation.x += (odom_vec_.at(i)).pose.pose.orientation.x;
                    avg_msg.pose.pose.orientation.y += (odom_vec_.at(i)).pose.pose.orientation.y;
                    avg_msg.pose.pose.orientation.z += (odom_vec_.at(i)).pose.pose.orientation.z;
                    avg_msg.pose.pose.orientation.w += (odom_vec_.at(i)).pose.pose.orientation.w;
                }
                avg_msg.pose.pose.position.x /= 3.0;
                avg_msg.pose.pose.position.y /= 3.0;
                avg_msg.pose.pose.position.z /= 3.0;
                avg_msg.pose.pose.orientation.x /= 3.0;
                avg_msg.pose.pose.orientation.y /= 3.0;
                avg_msg.pose.pose.orientation.z /= 3.0;
                avg_msg.pose.pose.orientation.w /= 3.0;

                odom_avg_pub_->publish(avg_msg);
                odom_vec_.clear();
            }
        }
        

        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
            imu_vec_.push_back(*msg);
            if(imu_vec_.size() == 20){
                sensor_msgs::msg::Imu avg_msg;
                avg_msg.angular_velocity.x = 0.0;
                avg_msg.angular_velocity.y = 0.0;
                avg_msg.angular_velocity.z = 0.0;
                avg_msg.linear_acceleration.x = 0.0;
                avg_msg.linear_acceleration.y = 0.0;
                avg_msg.linear_acceleration.z = 0.0;

                for(int i = 0; i < 20; i++){
                    avg_msg.angular_velocity.x += (imu_vec_.at(i)).angular_velocity.x; 
                    avg_msg.angular_velocity.y += (imu_vec_.at(i)).angular_velocity.y; 
                    avg_msg.angular_velocity.z += (imu_vec_.at(i)).angular_velocity.z; 
                    avg_msg.linear_acceleration.x += (imu_vec_.at(i)).linear_acceleration.x;
                    avg_msg.linear_acceleration.y += (imu_vec_.at(i)).linear_acceleration.y;
                    avg_msg.linear_acceleration.z += (imu_vec_.at(i)).linear_acceleration.z;
                }
                avg_msg.angular_velocity.x /= 20.0;
                avg_msg.angular_velocity.y /= 20.0;
                avg_msg.angular_velocity.z /= 20.0;
                avg_msg.linear_acceleration.x /= 20.0;
                avg_msg.linear_acceleration.y /= 20.0;
                avg_msg.linear_acceleration.z /= 20.0;

                imu_avg_pub_->publish(avg_msg);
                imu_vec_.clear();
            }
        }

        void odomAvgCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "ODOM: Pose: x: %lf; y: %lf; z: %lf\nOrientation: x: %lf; y: %lf; z: %lf; w: %lf",
             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, 
             msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        }

        void imuAvgCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "IMU: Angular Velocity: x: %lf; y: %lf; z: %lf\nLinear Acceleration: x: %lf; y:%lf; z: %lf\n",
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, 
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        }


        std::vector<nav_msgs::msg::Odometry> odom_vec_;
        std::vector<sensor_msgs::msg::Imu> imu_vec_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_avg_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_avg_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_avg_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_avg_pub_;

};



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}