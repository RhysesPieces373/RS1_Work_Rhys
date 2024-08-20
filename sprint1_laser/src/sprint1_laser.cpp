#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor() : Node("laser_scan_processor"){
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_nth", 10);
        n = rclcpp::Node::declare_parameter("n", 2);
    }

private:

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
        
        auto nth_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        nth_scan->ranges = std::vector<float>(scan->ranges.size(), 0.0);
        
        for(long unsigned int i = 0; i < scan->ranges.size() - 1; i+=n){
            if(i > 359){
                break;
            }
            nth_scan->ranges.at(i) = scan->ranges.at(i);
            
        }

        nth_scan->angle_min = scan->angle_min;
        nth_scan->angle_max = scan->angle_max;
        nth_scan->angle_increment = scan->angle_increment;

        scan_pub_->publish(*nth_scan);
        RCLCPP_INFO(this->get_logger(), "Published scan");
        
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    long int n;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}