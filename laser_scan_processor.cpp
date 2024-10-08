/**
 * @file laser_processing.cpp
 * @brief This file contains the LaserProcessing class which processes laser scan data to detect cylinders that have 30cm diameter
 *        and publishes them as markers for visualization in RViz.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>
#include <mutex>

/**
 * @brief The LaserProcessing class handles laser scan and odometry data, detects cylinders, 
 *        and publishes them as markers in RViz.
 */
class LaserProcessing : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the LaserProcessing class.
     */
    LaserProcessing() : Node("laser_processing_node") {
        RCLCPP_INFO(this->get_logger(), "Laser Processing Node started.");

        // Subscribe to the /scan topic to get laser scan data
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserProcessing::laser_callback, this, std::placeholders::_1)
        );

        // Subscribe to the /odom topic to get robot position and orientation
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LaserProcessing::odom_callback, this, std::placeholders::_1)
        );

        // Publisher for visualization markers (to visualize detected cylinders in RViz)
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_marker", 10);
    }

private:
    sensor_msgs::msg::LaserScan laserScan_;  ///< Latest laser scan data
    nav_msgs::msg::Odometry currentOdom_;    ///< Latest odometry data

    /**
     * @brief Callback function for the laser scan topic.
     * @param msg The laser scan message.
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        laserScan_ = *msg;

        // Vector to store detected cylinders
        std::vector<geometry_msgs::msg::Point> detectedCylinders;

        // Call the function to detect cylinders
        detectCylinder(detectedCylinders);

        // If cylinders are detected, log a message and publish a marker
        if (!detectedCylinders.empty()) {
            // Transform to world coordinates using odometry data
            geometry_msgs::msg::Point world_cylinder = transformToWorld(detectedCylinders.at(0));
            publishCylinderMarker(world_cylinder);
        }
    }

    /**
     * @brief Detects cylinders with a diameter of 30 cm from laser scan data.
     * @param detectedCylinders A reference to a vector where detected cylinder center points will be stored.
     * @return A vector of detected cylinder center points.
     */
    std::vector<geometry_msgs::msg::Point> detectCylinder(std::vector<geometry_msgs::msg::Point>& detectedCylinders) {
        sensor_msgs::msg::LaserScan laserScan = laserScan_;
        detectedCylinders.clear();  // Clear vector for current scan

        double segmentThreshold = 0.9;       ///< Threshold for distance between segmenting points
        double cylinderDiameter = 0.30;      ///< Diameter of the cylinder we want to detect (30 cm)
        double diameterTolerance = 0.05;     ///< Allowable tolerance for diameter comparison

        std::vector<std::vector<geometry_msgs::msg::Point>> segments;
        std::vector<geometry_msgs::msg::Point> currentSegment;

        // Segment the laser scan points based on proximity (distance between consecutive points)
        for (unsigned int i = 0; i < laserScan.ranges.size(); i++) {
            double range = laserScan.ranges.at(i);

            if (!std::isinf(range) && !std::isnan(range) && range < laserScan.range_max) {
                geometry_msgs::msg::Point point = polarToCart(i);  // Convert laser scan point to Cartesian coordinates

                if (currentSegment.empty()) {
                    currentSegment.push_back(point);
                } else {
                    geometry_msgs::msg::Point previousPoint = currentSegment.back();
                    double distance = std::hypot(point.x - previousPoint.x, point.y - previousPoint.y);

                    if (distance < segmentThreshold) {
                        currentSegment.push_back(point);  // Add to current segment
                    } else {
                        if (currentSegment.size() > 1) {
                            segments.push_back(currentSegment);  // Store valid segments
                        }
                        currentSegment.clear();
                        currentSegment.push_back(point);  // Start a new segment
                    }
                }
            }
        }

        if (currentSegment.size() > 1) {
            segments.push_back(currentSegment);  // Push the last segment if valid
        }

        // Process the segments to detect the cylinder
        for (auto& segment : segments) {
            if (segment.size() < 4) {
                continue;  // Skip too small segments
            }

            geometry_msgs::msg::Point firstPoint = segment.front();
            geometry_msgs::msg::Point lastPoint = segment.back();
            double segmentLength = std::hypot(lastPoint.x - firstPoint.x, lastPoint.y - firstPoint.y);

            if (std::abs(segmentLength - cylinderDiameter) <= diameterTolerance) {
                // Calculate the center point of the segment
                geometry_msgs::msg::Point center;
                center.x = (firstPoint.x + lastPoint.x) / 2.0;
                center.y = (firstPoint.y + lastPoint.y) / 2.0;
                detectedCylinders.push_back(center);  // Add detected cylinder's center point
            }
        }

        return detectedCylinders;
    }

    /**
     * @brief Converts polar coordinates (from the laser scan) to Cartesian coordinates.
     * @param index The index of the laser scan point.
     * @return The corresponding Cartesian coordinates.
     */
    geometry_msgs::msg::Point polarToCart(unsigned int index) {
        float angle = laserScan_.angle_min + laserScan_.angle_increment * index;
        float range = laserScan_.ranges.at(index);

        geometry_msgs::msg::Point cart;
        cart.x = static_cast<double>(range * cos(angle));
        cart.y = static_cast<double>(range * sin(angle));
        return cart;
    }

    /**
     * @brief Callback function for the odometry topic.
     * @param msg The odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        currentOdom_ = *msg;
    }

    /**
     * @brief Transforms the detected cylinder's local position to world coordinates.
     * @param local_point The detected cylinder's position in local (robot) coordinates.
     * @return The cylinder's position in world coordinates.
     */
    geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point& local_point) {
        geometry_msgs::msg::Point world_point;

        // Get the robot's position and yaw angle from the odometry message
        double robot_x = currentOdom_.pose.pose.position.x;
        double robot_y = currentOdom_.pose.pose.position.y;

        tf2::Quaternion q(
            currentOdom_.pose.pose.orientation.x,
            currentOdom_.pose.pose.orientation.y,
            currentOdom_.pose.pose.orientation.z,
            currentOdom_.pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Apply the transformation: rotate and translate the local point
        world_point.x = robot_x + local_point.x * cos(yaw) - local_point.y * sin(yaw);
        world_point.y = robot_y + local_point.x * sin(yaw) + local_point.y * cos(yaw);
        world_point.z = 0.0;  // Assuming the cylinder is on the ground

        return world_point;
    }

    /**
     * @brief Publishes a marker in RViz for the detected cylinder.
     * @param cylinder_position The position of the detected cylinder.
     */
    void publishCylinderMarker(const geometry_msgs::msg::Point& cylinder_position) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";  ///< Frame ID for the marker
        marker.header.stamp = this->get_clock()->now();

        marker.ns = "cylinder_detection";  ///< Namespace for the marker
        marker.id = 0;  ///< Marker ID

        marker.type = visualization_msgs::msg::Marker::CYLINDER;  ///< Marker type
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = cylinder_position.x;
        marker.pose.position.y = cylinder_position.y;
        marker.pose.position.z = 0.0;  // Assuming it's on the ground

        marker.pose.orientation.w = 1.0;  // No rotation needed

        marker.scale.x = 0.30;  // 30 cm diameter
        marker.scale.y = 0.30;
        marker.scale.z = 1.0;  // 1 meter height

        marker.color.r = 0.0f;  // Blue color
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;  // Fully opaque

        marker_publisher_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;  ///< Laser scan subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;      ///< Odometry data subscriber
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_; ///< Marker publisher
};

/**
 * @brief Main function to start the LaserProcessing node.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return Execution status.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserProcessing>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


