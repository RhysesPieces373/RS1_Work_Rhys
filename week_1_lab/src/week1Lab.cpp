#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber()
    : Node("subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 1, std::bind(&Subscriber::topic_callback, this, _1));

      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/modified_image", 1);
      
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const
    {
      
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg);
      } 
      catch (cv_bridge::Exception& e){
        RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      
      RCLCPP_INFO(this->get_logger(), "%d rows, %d columns", cv_ptr->image.rows, cv_ptr->image.cols);
      cv::circle(cv_ptr->image, cv::Point((cv_ptr->image.cols)/2, (cv_ptr->image.rows)/2), 50, CV_RGB(255,0,0), 5);

      publisher_->publish(*(cv_ptr->toImageMsg()));
    }
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}