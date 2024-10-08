#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // tf2 conversions
#include <tf2/exceptions.h>
#include <cmath>
#include <vector>

class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector()
    : Node("cylinder_detector"), cylinder_diameter_(0.30), 
    tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),  // Correct initialization of tf_buffer_
    tf_listener_(tf_buffer_)                                    // Initialize tf_listener_ with tf_buffer_
    {
        // Subscribe to LaserScan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));

        // Publisher for visualization marker
        cyl_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);

        // Publisher for pose (optional)
        cyl_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cylinder_pose", 10);

        // Initialize the TF broadcaster
        tf_BC_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Cylinder parameter
        cylinder_radius_ = cylinder_diameter_ / 2.0;
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::vector<float> ranges = scan_msg->ranges;
        size_t n = ranges.size();

        // Laser scan parameters
        double angle_min = scan_msg->angle_min;
        double angle_increment = scan_msg->angle_increment;

        // Vectors to store Cartesian coordinates of laser points
        std::vector<double> lsX, lsY;

        // Convert polar coordinates to Cartesian coordinates
        for (size_t i = 0; i < n; ++i)
        {
            double angle = angle_min + i * angle_increment;
            double x = ranges[i] * std::cos(angle);
            double y = ranges[i] * std::sin(angle);
            lsX.push_back(x);
            lsY.push_back(y);
        }

        // Detect cylindrical objects in the laser scan data
        int cylinder_idx = detectCylinderObject(lsX, lsY);
        if (cylinder_idx != -1)
        {
            // Calculate the mean position of the cylinder in laser frame
            double cylinder_x = lsX[cylinder_idx];
            double cylinder_y = lsY[cylinder_idx];

            // Publish the detected cylinder's marker
            publishCylinderMarker(cylinder_x, cylinder_y);

            // Publish a TF transform for the cylinder
            publishCylinderTF(cylinder_x, cylinder_y);
        }
    }

    int detectCylinderObject(const std::vector<double>& xs, const std::vector<double>& ys)
    {
        size_t n = xs.size();
        std::vector<double> distances(n - 1);

        // Calculate distances between consecutive points
        for (size_t i = 0; i < n - 1; ++i)
        {
            distances[i] = std::sqrt(std::pow(xs[i + 1] - xs[i], 2) + std::pow(ys[i + 1] - ys[i], 2));
        }
// Threshold to consider consecutive points as part of the same cluster
        double distance_threshold = 0.15;  // 15 cm

        // Loop over the points and find clusters
        for (size_t i = 0; i < n - 1; ++i)
        {
            std::vector<size_t> cluster_indices;  // To store indices of points in the current cluster

            // Start a new cluster
            cluster_indices.push_back(i);

            // Continue adding points to the cluster as long as the distance is below the threshold
            while (i < n - 1 && distances[i] < distance_threshold)
            {
                cluster_indices.push_back(i + 1);  // Add the next point to the cluster
                ++i;  // Move to the next point
            }

            // Now calculate the total "width" of the cluster (in terms of its x and y coordinates)
            double cluster_x_min = xs[cluster_indices.front()];
            double cluster_x_max = xs[cluster_indices.back()];

            double cluster_y_min = ys[cluster_indices.front()];
            double cluster_y_max = ys[cluster_indices.back()];

            double cluster_width = std::sqrt(std::pow(cluster_x_max - cluster_x_min, 2) + std::pow(cluster_y_max - cluster_y_min, 2));

            // If the detected cluster width is within the expected range for the cylinder (25 cm to 30 cm)
            if (cluster_width >= 0.25 && cluster_width <= 0.30)
            {
                // Return the index of the first point in the cluster (this can be used as the "detected" cylinder)
                return cluster_indices.front();
            }
        }
        return -1;  // No cylinder found
    }

    void publishCylinderMarker(double x, double y)
    {
        // Create RViz Marker from visualisation msgs
        visualization_msgs::msg::Marker cylMarker;
        cylMarker.ns = "cylinder";
        cylMarker.id = 0;
        cylMarker.type = visualization_msgs::msg::Marker::CYLINDER;
        cylMarker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale and color of the cylinder marker
        cylMarker.scale.x = cylinder_diameter_;
        cylMarker.scale.y = cylinder_diameter_;
        cylMarker.scale.z = 1.0;
        cylMarker.color.r = 1.0;
        cylMarker.color.g = 0.0;
        cylMarker.color.b = 0.0;
        cylMarker.color.a = 1.0;

        // Transform the position from base_link to map (can also be odom)
        geometry_msgs::msg::TransformStamped tfStamped;

        try
        {
            // Lookup the transform from base_link to map (can also be odom)
            tfStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            // Apply the transform to the cylinder position (x, y)
            geometry_msgs::msg::PointStamped cylinderBasePoint;
            cylinderBasePoint.header.frame_id = "base_link";
            cylinderBasePoint.point.x = x;
            cylinderBasePoint.point.y = y;
            cylinderBasePoint.point.z = 0;

            geometry_msgs::msg::PointStamped cylinderMapPoint;
            tf2::doTransform(cylinderBasePoint, cylinderMapPoint, tfStamped);

            // Set the actual position of the marker
            cylMarker.pose.position.x = cylinderMapPoint.point.x;
            cylMarker.pose.position.y = cylinderMapPoint.point.y;
            cylMarker.pose.position.z = 0.0;  // Assuming cylinder is on the ground
            cylMarker.pose.orientation.w = 1.0;

            // Set the frame to correct frame id: map (or odom)
            cylMarker.header.frame_id = "map";  // Change this to odom if necessary
            cylMarker.header.stamp = this->get_clock()->now();

            // Publish the cylinder marker
            cyl_marker_pub_->publish(cylMarker);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        }
    }

    void publishCylinderTF(double x, double y)
    {
        // Create and broadcast a TransformStamped for the detected cylinder
        geometry_msgs::msg::TransformStamped tfStamped;

        tfStamped.header.stamp = this->get_clock()->now();
        tfStamped.header.frame_id = "base_link";
        tfStamped.child_frame_id = "cylinder";
        tfStamped.transform.translation.x = x;
        tfStamped.transform.translation.y = y;
        tfStamped.transform.translation.z = 0.0;
        tfStamped.transform.rotation.w = 1.0;

        tf_BC_->sendTransform(tfStamped);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cyl_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cyl_pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_BC_;

    tf2_ros::Buffer tf_buffer_; 
    tf2_ros::TransformListener tf_listener_; 

    double cylinder_diameter_; // Known diameter of the placed cylinder obstacle
    double cylinder_radius_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}