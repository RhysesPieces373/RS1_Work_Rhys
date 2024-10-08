#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <iostream>

class MapPublishNode : public rclcpp::Node
{
    public:
        MapPublishNode()
        : Node("map_publish_node"), doneOnce_(false)
        {
            // subscribe to /map topic, convert to cv::Mat (from occupancyGrid)  
            // show both images separately and overlay them with different colours

            image = cv::imread("map.pgm", cv::IMREAD_GRAYSCALE);
            
            std::cout << "After loading" << std::endl;
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapPublishNode::mapCallback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&MapPublishNode::imgOverlay, this));
            
        }

    private:

        void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg){
            mapMsg_ = mapMsg;
            
        }

        void imgOverlay(){
            cv::imshow("Ground Truth", image);
            cv::waitKey(1);
            if(!doneOnce_ && mapMsg_ != NULL){
                
                occupancyGridToImage(mapMsg_);
                cv::imshow("SLAM Map", m_temp_img);
                cv::waitKey(1);
                std::cout << m_temp_img.cols << std::endl;
                double x_scale = (double)m_temp_img.cols/ (double)image.cols;
                double y_scale = (double)m_temp_img.rows/ (double)image.rows;
                std::cout << "X: " << x_scale << ", " << "Y: " << y_scale << std::endl;
                cv::resize(image, image, cv::Size(), x_scale, y_scale);
                cv::imshow("Ground Truth", image);
                cv::imshow("SLAM Map", m_temp_img);
                cv::waitKey(1);

                overlayImages(image, m_temp_img);

                cv::imshow("Overlayed", overlayImg_);
                cv::waitKey(1);

                doneOnce_ = true;
            }
            
        }

        void overlayImages(cv::Mat base, cv::Mat overlay){
            int row, col;

            base.copyTo(overlayImg_);
            for(row = 0; row < overlayImg_.rows; row++){
                for(col = 0; col < overlayImg_.cols; col++){
                    if(overlay.at<uchar>(row, col) == 0){
                        overlayImg_.at<uchar>(row, col) = 180;
                    }
                }
            }
        }

        // Convert occupancy grid to binary image
        void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
        {
            int grid_data;
            unsigned int row, col, val;

            m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

            std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;

            for (row = 0; row < grid->info.height; row++) {
                for (col = 0; col < grid->info.width; col++) {
                    grid_data = grid->data[row * grid->info.width + col];
                    if (grid_data != -1) {
                        val = 255 - (255 * grid_data) / 100;
                        val = (val == 0) ? 0 : 255;
                        m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                    } else {
                        m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                    }
                }
            }
        }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr im_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    cv::Mat image;
    cv::Mat m_temp_img;
    cv::Mat overlayImg_;
    nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg_;

    rclcpp::TimerBase::SharedPtr timer_; 

    bool doneOnce_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPublishNode>());
    rclcpp::shutdown();
    return 0;
}