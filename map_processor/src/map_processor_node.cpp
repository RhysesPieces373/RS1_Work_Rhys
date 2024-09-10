#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::cout << "scanCallback" << std::endl;
        // cv::Mat tmp_col_img = m_MapColImage.clone();
        // cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        // cv::imshow(WINDOW1, tmp_col_img);
        cv::waitKey(1);    
    }
    

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        occupancyGridToImage(mapMsg);

        cv::Mat tmp_col_img = m_MapColImage.clone();

        cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW1, tmp_col_img);
        cv::waitKey(1);
    }

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
                val = (val == 0) ? 255 : 0;
                m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
            } else {
                m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
            }
        }
    }

    map_scale_ = grid->info.resolution;
    origin_x = grid->info.origin.position.x;
    origin_y = grid->info.origin.position.y;
    size_x = grid->info.width;
    size_y = grid->info.height;

    cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                0, 1, 0,
                                0, 0, 0);
    cv::erode(m_temp_img, m_MapBinImage, kernel);

    m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
    cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

    std::cout << "Occupancy grid map converted to a binary image\n";

    // // Display the image to verify
    // cv::imshow("Occupancy Grid", m_MapColImage);
    // cv::waitKey(1);
}


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;


    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;

    const std::string WINDOW1 = "Map Image";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

