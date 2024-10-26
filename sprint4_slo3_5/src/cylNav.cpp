
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  
#include <tf2/exceptions.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @class CylinderDetector
 * @brief A ROS 2 node for detecting cylindrical objects from laser scan data.
 * 
 * This node listens to laser scan data, processes it to detect cylinders
 * within a certain size range (in this case 30cm), and publishes the detected cylinder's position with
 * a marker and a pose. It also manages the robot's navigation by subscribing to Nav2 goals, canceling them when a
 * cylinder is detected, performing a circular motion around the cylinder, and then resuming the goal.
 */
class CylinderDetector : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the CylinderDetector node.
     * 
     * Initializes the node, sets up the laser scan subscriber, marker publisher, pose publisher,
     * and transform broadcaster. It also subscribes to Nav2 goals and manages them accordingly.
     */
    CylinderDetector() : Node("cylinder_detector"), tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),  
      tf_listener_(tf_buffer_), navigating_around_cylinder_(false), circle_time_(0), original_position_saved_(false)                                    
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cylinder_pose", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribe to Nav2 goals and initialize goal publisher
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&CylinderDetector::goalCallback, this, std::placeholders::_1));
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        // Timer for controlling the loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&CylinderDetector::controlLoop, this));

        cylinder_diameter_ = 0.30;  ///< Diameter of the cylinder to detect (in meters)
        cylinder_radius_ = cylinder_diameter_ / 2.0; ///< Radius of the cylinder to detect (in meters)
        circle_radius_ = cylinder_radius_ + 0.7;
    }

private:
    /**
     * @brief Callback function for processing laser scan data.
     * 
     * Converts the laser scan data from polar coordinates to Cartesian coordinates and attempts to
     * detect a cylinder. If a cylinder is detected, it stops the robot's current goal, moves towards
     * the cylinder, and initiates the circular motion.
     * 
     * @param scan_msg The laser scan message received from the `/scan` topic.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::vector<float> ranges = scan_msg->ranges;
        size_t n = ranges.size();

        double angle_min = scan_msg->angle_min;
        double angle_increment = scan_msg->angle_increment;

        std::vector<double> xs, ys;

        // Convert polar coordinates to Cartesian coordinates
        for (size_t i = 0; i < n; ++i)
        {
            double angle = angle_min + i * angle_increment;
            double x = ranges[i] * std::cos(angle);
            double y = ranges[i] * std::sin(angle);
            xs.push_back(x);
            ys.push_back(y);
        }

        // Detect the cylinder
        int cylinder_idx = detectCylinder(xs, ys);
        if (cylinder_idx != -1 && !navigating_around_cylinder_ && !moving_to_cylinder_)
        {
            cylinder_position_.x = xs[cylinder_idx];
            cylinder_position_.y = ys[cylinder_idx];

            RCLCPP_INFO(this->get_logger(), "Cylinder detected at (%f, %f)", cylinder_position_.x, cylinder_position_.y);

            // Publish marker and TF for visualization
            publishCylinderMarker(cylinder_position_.x, cylinder_position_.y);
            publishCylinderTF(cylinder_position_.x, cylinder_position_.y);

            // Stop moving towards the Nav2 goal and start moving towards the cylinder
            cancelCurrentNavGoal();  // Cancel the Nav2 goal
            moving_to_cylinder_ = true;
            stopRobot();  // Stop the robot temporarily
        }
    }

    /**
     * @brief Detects a cylinder from Cartesian coordinates.
     * 
     * Groups the laser scan points into clusters based on proximity. If a cluster width matches the 
     * expected size of a cylinder (25 cm to 30 cm), it is detected.
     * 
     * @param xs The x-coordinates of the laser scan points.
     * @param ys The y-coordinates of the laser scan points.
     * @return The index of the detected cylinder, or -1 if no cylinder is detected.
     */
    int detectCylinder(const std::vector<double>& xs, const std::vector<double>& ys)
    {
        size_t n = xs.size();
        std::vector<double> distances(n - 1);

        // Calculate distances between consecutive points
        for (size_t i = 0; i < n - 1; ++i)
        {
            distances[i] = std::sqrt(std::pow(xs[i + 1] - xs[i], 2) + std::pow(ys[i + 1] - ys[i], 2));
        }

        double distance_threshold = 0.15;

        // Loop over the points and find clusters
        for (size_t i = 0; i < n - 1; ++i)
        {
            std::vector<size_t> cluster_indices;  // To store indices of points in the current cluster
            cluster_indices.push_back(i);

            // Continue adding points to the cluster as long as the distance is below the threshold
            while (i < n - 1 && distances[i] < distance_threshold)
            {
                cluster_indices.push_back(i + 1);
                ++i;
            }

            // Calculate the total width of the cluster
            double cluster_x_min = xs[cluster_indices.front()];
            double cluster_x_max = xs[cluster_indices.back()];
            double cluster_y_min = ys[cluster_indices.front()];
            double cluster_y_max = ys[cluster_indices.back()];

            double cluster_width = std::sqrt(std::pow(cluster_x_max - cluster_x_min, 2) + std::pow(cluster_y_max - cluster_y_min, 2));

            // Check if the cluster width matches the cylinder's width (25 cm to 30 cm)
            if (cluster_width >= 0.25 && cluster_width <= 0.30)
            {
                return cluster_indices.front();  // Return the index of the detected cylinder
            }
        }

        return -1;
    }

    /**
     * @brief Publishes a marker to visualize the detected cylinder in RViz.
     * 
     * @param x The x-coordinate of the detected cylinder.
     * @param y The y-coordinate of the detected cylinder.
     */
    void publishCylinderMarker(double x, double y)
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = cylinder_diameter_;
        marker.scale.y = cylinder_diameter_;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        geometry_msgs::msg::TransformStamped transformStamped;

        try
        {
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            geometry_msgs::msg::PointStamped cylinder_point_in_base;
            cylinder_point_in_base.header.frame_id = "base_link";
            cylinder_point_in_base.point.x = x;
            cylinder_point_in_base.point.y = y;

            geometry_msgs::msg::PointStamped cylinder_point_in_map;
            tf2::doTransform(cylinder_point_in_base, cylinder_point_in_map, transformStamped);

            marker.pose.position.x = cylinder_point_in_map.point.x;
            marker.pose.position.y = cylinder_point_in_map.point.y;
            marker.pose.orientation.w = 1.0;

            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();

            marker_pub_->publish(marker);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        }
    }

    /**
     * @brief Publishes a transform for the detected cylinder.
     * 
     * @param x The x-coordinate of the detected cylinder.
     * @param y The y-coordinate of the detected cylinder.
     */
    void publishCylinderTF(double x, double y)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "cylinder";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    /**
     * @brief Main control loop for navigating towards the cylinder, circling it, and resuming the original goal.
     */
    void controlLoop()
    {
        if (moving_to_cylinder_)
        {
            moveToCylinder();  // Move towards the cylinder
        }
        else if (navigating_around_cylinder_)
        {
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.3;   // Set linear velocity
            cmd_vel_msg.angular.z = 0.3;  // Set angular velocity for circular motion
            vel_pub_->publish(cmd_vel_msg);

            circle_time_ += 0.1;  // Increment time (assuming control loop runs every 0.1s)

            if (circle_time_ >= 6.28)  // Complete the circle
            {
                navigating_around_cylinder_ = false;
                stopRobot();

                RCLCPP_INFO(this->get_logger(), "Circle complete, resuming navigation...");
                resumeNavigation();  // Resume the original Nav2 goal
            }
        }
    }

    /**
     * @brief Function to stop the robot.
     */
    void stopRobot()
    {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        vel_pub_->publish(stop_msg);
    }

    /**
     * @brief Publishes a "null" goal to stop the Nav2 navigation.
     */
    void cancelCurrentNavGoal()
    {
        geometry_msgs::msg::PoseStamped null_goal;
        null_goal.header.stamp = this->get_clock()->now();
        null_goal.header.frame_id = "map";

        // Use the current robot position to stop
        null_goal.pose.position.x = current_robot_position_.x;
        null_goal.pose.position.y = current_robot_position_.y;
        null_goal.pose.orientation.w = 1.0;

        goal_pub_->publish(null_goal);  // Publish the "null" goal
        RCLCPP_INFO(this->get_logger(), "Published 'null' goal to stop the robot.");
    }

    /**
     * @brief Republish the stored goal to resume Nav2 navigation.
     */
    void resumeNavigation()
    {
        RCLCPP_INFO(this->get_logger(), "Republishing the original goal...");
        goal_pub_->publish(stored_goal_);
    }

    /**
     * @brief Callback function to store the current Nav2 goal.
     */
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
    {
        stored_goal_ = *goal_msg;
        RCLCPP_INFO(this->get_logger(), "Goal stored: (%f, %f)", stored_goal_.pose.position.x, stored_goal_.pose.position.y);
    }

    /**
     * @brief Moves the robot towards the cylinder.
     */
    void moveToCylinder()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            double robot_x = transformStamped.transform.translation.x;
            double robot_y = transformStamped.transform.translation.y;

            double distance_to_cylinder = std::sqrt(
                std::pow(cylinder_position_.x - robot_x, 2) + std::pow(cylinder_position_.y - robot_y, 2));

            if (distance_to_cylinder <= 1.0)  // If the robot is close enough to the cylinder
            {
                RCLCPP_INFO(this->get_logger(), "Reached cylinder, starting to circle...");
                moving_to_cylinder_ = false;
                navigating_around_cylinder_ = true;
                circle_time_ = 0;  // Reset the time for circling
            }
            else
            {
                // Continue moving towards the cylinder
                geometry_msgs::msg::Twist move_msg;
                move_msg.linear.x = 0.2;  // Adjust speed as needed
                vel_pub_->publish(move_msg);
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get current position: %s", ex.what());
        }
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    geometry_msgs::msg::PoseStamped stored_goal_;  // Store the Nav2 goal
    geometry_msgs::msg::Point current_robot_position_;  // Store the robot's current position
    geometry_msgs::msg::Point cylinder_position_;  // Store the cylinder position

    bool navigating_around_cylinder_;
    bool moving_to_cylinder_;
    bool original_position_saved_;
    double circle_time_;
    double cylinder_diameter_;
    double cylinder_radius_;
    double circle_radius_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}