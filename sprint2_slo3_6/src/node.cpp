#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <mutex>

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode()
    : Node("map_processor_node"), initial_pose_received_(false), map_create_(false), image_size_(200)
    {
        // Subscription to the occupancy grid map
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

        // Subscription to the laser scan
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapProcessorNode::scanCallback, this, std::placeholders::_1));

        // Subscription to the odometry
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapProcessorNode::odomCallback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);
    }

private:
    // Callback to handle odometry messages and capture the initial pose
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {   
        current_pose_ = odomMsg->pose.pose;  // Store the initial pose 
        if(map_create_){
            robot_x = static_cast<int>((current_pose_.position.x - grid_->info.origin.position.x)/grid_->info.resolution);
            robot_y = static_cast<int>((current_pose_.position.y - grid_->info.origin.position.y)/grid_->info.resolution);
            xStart = std::max(0,robot_x - half_window);
            yStart = std::max(0,robot_y - half_window);
            xEnd = std::min(static_cast<int>(grid_->info.width), robot_x + half_window);
            yEnd = std::min(static_cast<int>(grid_->info.height), robot_y + half_window);
        
            cv::Rect roi(xStart, yStart, xEnd - xStart, yEnd - yStart);
            cv::Mat local_map = edges1(roi);
            edges1_masked_ = local_map.clone();

            cv::Mat image = cv::Mat::zeros(image_size_, image_size_, CV_8UC1); 
            for(int row = 0; row < edges1_masked_.rows; row++){
                for(int col = 0; col < edges1_masked_.cols; col++){
                    int x = static_cast<int>(col) + image_size_ / 2;
                    int y = static_cast<int>(row) + image_size_ / 2;
                    if (x >= 0 && x < image.cols && y >= 0 && y < image.cols && edges1_masked_.at<uchar>(row, col) == 255) {
                        image.at<uchar>(y, x) = 255;
                    }
                }
            }
            
            edges1_upscaled_ = image.clone();
            cv::rotate(edges1_upscaled_, edges1_upscaled_, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::imshow("Cropped Edge Map", edges1_upscaled_);
            cv::waitKey(1);
        }
    }

    // Callback for laser scan messages
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        // Generate image from laser scan data (img2)
        cv::Mat img2 = laserScanToMat(msg);
        if(map_create_){
        // Compare the binary map image (img1) with the laser scan image (img2)

            calculateYawChange(edges1_upscaled_, img2);
            moveRobot();
        }

    }

    // Callback for occupancy grid map messages
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        grid_ = mapMsg;

        // Convert occupancy grid to binary image
        occupancyGridToImage(mapMsg);
        map_create_ = true;

        std::cout << "mapCallback finished" << std::endl;
        
    }

    void moveRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.3;  // Rotate with some angular velocity
        vel_pub_->publish(twist_msg);

        // Sleep for a while to allow the robot to rotate
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Stop rotation
        twist_msg.angular.z = 0.0;
        vel_pub_->publish(twist_msg);
    }


    // Function to compare the map (binary image) with the laser scan image and calculate yaw change
    void calculateYawChange(const cv::Mat& img1, const cv::Mat& img2) {
        std::vector<cv::Point2f> srcPoints, dstPoints;

        detectAndMatchFeatures(img1, img2, srcPoints, dstPoints);
        RCLCPP_INFO(this->get_logger(), "src= %ld", srcPoints.size());
        RCLCPP_INFO(this->get_logger(), "dst= %ld", dstPoints.size());
        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle (yaw) from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;  // Convert to degrees
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                            std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        // Create ORB detector
        cv::Ptr<cv::ORB> orb = cv::ORB::create(4000);  // Increased number of features for better detection
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        
        // Detect keypoints and compute descriptors for both images
        cv::Canny(img2, img2, 30, 90);  // Adjust thresholds if necessary
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);
        
        cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create();
        std::vector<cv::KeyPoint> fast_keypoints1, fast_keypoints2;
        fast->detect(img1, fast_keypoints1);
        keypoints1.insert(keypoints1.end(), fast_keypoints1.begin(), fast_keypoints1.end());
        fast->detect(img2, fast_keypoints2);
        keypoints2.insert(keypoints2.end(), fast_keypoints2.begin(), fast_keypoints2.end());

        cv::Ptr<cv::ORB> orb2 = cv::ORB::create();
        orb2->compute(img1, keypoints1, descriptors1);
        orb2->compute(img2, keypoints2, descriptors2);

        // Visualize the keypoints on both images
        cv::Mat img1_keypoints, img2_keypoints;
        cv::drawKeypoints(img1, keypoints1, img1_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::drawKeypoints(img2, keypoints2, img2_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

        // Display the keypoints
        cv::imshow("Map Keypoints (img1)", img1_keypoints);
        cv::imshow("Laser Scan Keypoints (img2)", img2_keypoints);
        cv::waitKey(1);  // Pause for visualization

        // Perform feature matching
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);


        // RANSAC 
        std::vector<cv::DMatch> goodMatches;

        goodMatches.clear();
        // Convert keypoints into Point2f
        std::vector<cv::Point2f> points1, points2;
        for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it)
        {
            // Get the position of left keypoints
            float x= keypoints1[it->queryIdx].pt.x;
            float y= keypoints1[it->queryIdx].pt.y;
            points1.push_back(cv::Point2f(x,y));
            // Get the position of right keypoints
            x= keypoints2[it->trainIdx].pt.x;
            y= keypoints2[it->trainIdx].pt.y;
            points2.push_back(cv::Point2f(x,y));
        }
        // Compute F matrix using RANSAC
        std::vector<uchar> inliers(points1.size(),0);
        cv::Mat fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2),inliers,cv::FM_RANSAC,1,0.99); // confidence probability
        // extract the surviving (inliers) matches
        std::vector<uchar>::const_iterator itIn= inliers.begin();
        std::vector<cv::DMatch>::const_iterator itM= matches.begin();
        // for all matches
        for ( ;itIn!= inliers.end(); ++itIn, ++itM)
        {
            if (*itIn)
            { // it is a valid match
                goodMatches.push_back(*itM);
            }
        }

        // Sort matches based on distance (lower distance means better match)
        std::sort(goodMatches.begin(), goodMatches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (15% of total matches)
        size_t numGoodMatches = static_cast<size_t>(goodMatches.size() * 0.30);

        // Keep only the best matches (top 15%)
        std::vector<cv::DMatch> gooderMatches(goodMatches.begin(), goodMatches.begin() + numGoodMatches);

        // Populate srcPoints and dstPoints with the best matches
        for (const auto& match : gooderMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }

        // (Optional) Visualize matches
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, gooderMatches, img_matches);
        cv::imshow("Feature Matches", img_matches);
        cv::waitKey(1);  // Pause for visualization
    }//////////////////////////////////////////////

    // Convert laser scan data to an OpenCV image (img2)
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        cv::Mat image = cv::Mat::zeros(image_size_, image_size_, CV_8UC1);  // Use binary map size

        float max_range = scan->range_max;
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * image_size_ / (2 * max_range)) + image_size_ / 2;
                int y = static_cast<int>((range * sin(angle)) * image_size_ / (2 * max_range)) + image_size_ / 2;
                if (x >= 0 && x < image_size_ && y >= 0 && y < image_size_) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
        return image;
    }

    
    //scale up using 2 ogti functinos. one runs at the start for full map, one runs each time movement occurs to generate local map data for each window. reducing process power requirements. 
    // Convert occupancy grid to binary image
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        robot_x = static_cast<int>((current_pose_.position.x - grid->info.origin.position.x)/grid->info.resolution);
        robot_y = static_cast<int>((current_pose_.position.y - grid->info.origin.position.y)/grid->info.resolution);
        xStart = std::max(0,robot_x - half_window);
        yStart = std::max(0,robot_y - half_window);
        xEnd = std::min(static_cast<int>(grid->info.width), robot_x + half_window);
        yEnd = std::min(static_cast<int>(grid->info.height), robot_y + half_window);

        //std::cout << robot_x << robot_y << xStart << yStart << std::endl


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
        
        // Store the binary map image (img1) for comparison with the laser scan image
        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                0, 1, 0,
                                0, 0, 0);

        cv::erode(m_temp_img, m_MapBinImage, kernel);
        
        cv::Canny(m_MapBinImage, edges1, 30, 90);  // Adjust thresholds if necessary

        
        cv::Rect roi(xStart, yStart, xEnd - xStart, yEnd - yStart);
        cv::Mat local_map = edges1(roi);
        edges1_masked_ = local_map.clone();

        cv::Mat image = cv::Mat::zeros(image_size_, image_size_, CV_8UC1); 
        for(row = 0; row < edges1_masked_.rows; row++){
            for(col = 0; col < edges1_masked_.cols; col++){
                int x = static_cast<int>(col) + image_size_ / 2;
                int y = static_cast<int>(row) + image_size_ / 2;
                if (x >= 0 && x < image.cols && y >= 0 && y < image.cols && edges1_masked_.at<uchar>(row, col) == 255) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        
        edges1_upscaled_ = image.clone();

        cv::rotate(m_MapBinImage, m_MapBinImage, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(edges1_upscaled_, edges1_upscaled_, cv::ROTATE_90_COUNTERCLOCKWISE);

        std::cout << "Occupancy grid map converted to a binary image\n";
        cv::imshow("Map", m_MapBinImage);
        cv::imshow("Cropped Edge Map", edges1_upscaled_);
        cv::waitKey(1);
    }

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    // Images
    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;  // Binary image from occupancy grid (img1)
    cv::Mat m_MapColImage;  // Color image for display
    cv::Mat edges1;
    cv::Mat edges1_masked_;
    cv::Mat edges1_upscaled_;

    // Map and pose properties

    int image_size_;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;
    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
   
    std::mutex odom_mutex_;
    bool initial_pose_received_;
    bool map_create_;
    double angle_difference_;  // Yaw angle difference (in degrees)

    int window_size = 60;
    int half_window = window_size/2;
    int robot_x;
    int robot_y;

    int xStart;
    int yStart;
    int xEnd;
    int yEnd;

    const std::string WINDOW1 = "Map Image";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

