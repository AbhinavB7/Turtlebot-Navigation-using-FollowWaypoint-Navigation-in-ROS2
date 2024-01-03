/**
 * @file broadcaster.hpp
 * @brief Header file for the Broadcaster class
 */
#pragma once


#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <limits>
#include "tf2_ros/static_transform_broadcaster.h"


using namespace std::chrono_literals;

/**
 * @class Broadcaster
 * @brief node to broadcast transforms for Aruco markers and batteries
 */
class Broadcaster : public rclcpp::Node
{
public:
    Broadcaster(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Broadcaster started");

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);

        // Define the Quality of Service profile for the subscriptions
        auto qos_profile = rclcpp::SensorDataQoS();

        // Create a subscription to the "/mage/camera1/image" topic
        Camera1subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera1/image", qos_profile, std::bind(&Broadcaster::camera1_sub_cb, this, std::placeholders::_1));

        // Create a subscription to the "/mage/camera2/image" topic
        Camera2subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera2/image", qos_profile, std::bind(&Broadcaster::camera2_sub_cb, this, std::placeholders::_1));

        // Create a subscription to the "/mage/camera3/image" topic
        Camera3subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera3/image", qos_profile, std::bind(&Broadcaster::camera3_sub_cb, this, std::placeholders::_1));

        // Create a subscription to the "/mage/camera4/image" topic
        Camera4subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera4/image", qos_profile, std::bind(&Broadcaster::camera4_sub_cb, this, std::placeholders::_1));

        // Create a subscription to the "/mage/camera5/image" topic
        Camera5subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera5/image", qos_profile, std::bind(&Broadcaster::camera5_sub_cb, this, std::placeholders::_1));

        // Create a subscription to the clock topic
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&Broadcaster::sub_clock_cb, this, std::placeholders::_1));

        // initialize a static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // timer to publish the transform
        static_broadcast_timer_ = this->create_wall_timer(
        10s,
        std::bind(&Broadcaster::static_broadcast_timer_cb_, this));
    }


private:
    
    /**
     * @brief Storing transforms
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /**
     * @brief Transform listener
     */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    /**
     * @brief Current Time
     */
    rclcpp::Time current_time_;

    /**
     * @brief Subscription to clock topic
     */
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

    /**
     * @brief Callback function for the clock subscription
     *
     * @param msg Message received from the clock topic
     */
    void sub_clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    /**
     * @brief Subscription to the "/mage/camera1/image" topic
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr Camera1subscription_;
    
    /**
     * @brief Callback function for the camera1 subscription
     *
     * @param msg Message received from the "/mage/advanced_logical_camera/image" topic
     */
    void camera1_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Subscription to the "/mage/camera2/image" topic
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr Camera2subscription_;
    
    /**
     * @brief Callback function for the camera2 subscription
     *
     * @param msg Message received from the "/mage/advanced_logical_camera/image" topic
     */
    void camera2_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Subscription to the "/mage/camera3/image" topic
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr Camera3subscription_;
    
    /**
     * @brief Callback function for the camera3 subscription
     *
     * @param msg Message received from the "/mage/advanced_logical_camera/image" topic
     */
    void camera3_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Subscription to the "/mage/camera4/image" topic
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr Camera4subscription_;
    
    /**
     * @brief Callback function for the camera4 subscription
     *
     * @param msg Message received from the "/mage/advanced_logical_camera/image" topic
     */
    void camera4_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Subscription to the "/mage/camera5/image" topic
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr Camera5subscription_;
    
    /**
     * @brief Callback function for the camera5 subscription
     *
     * @param msg Message received from the "/mage/advanced_logical_camera/image" topic
     */
    void camera5_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);


     /*!< Static broadcaster object */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    /*!< Wall timer object for the static broadcaster*/
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void static_broadcast_timer_cb_();

    /**
     * @brief TransformStamped message for part detected by camera1
     */
    geometry_msgs::msg::TransformStamped camera1_static_transform_stamped_;

    /**
     * @brief TransformStamped message for part detected by camera2
     */
    geometry_msgs::msg::TransformStamped camera2_static_transform_stamped_;

    /**
     * @brief TransformStamped message for part detected by camera3
     */
    geometry_msgs::msg::TransformStamped camera3_static_transform_stamped_;

    /**
     * @brief TransformStamped message for part detected by camera4
     */
    geometry_msgs::msg::TransformStamped camera4_static_transform_stamped_;

    /**
     * @brief TransformStamped message for part detected by camera5
     */
    geometry_msgs::msg::TransformStamped camera5_static_transform_stamped_;

    /**
     * @brief TransformStamped message for camera1 detected by world
     */
    geometry_msgs::msg::TransformStamped frame1_static_transform_stamped_;

    /**
     * @brief TransformStamped message for camera2 detected by world
     */
    geometry_msgs::msg::TransformStamped frame2_static_transform_stamped_;

    /**
     * @brief TransformStamped message for camera3 detected by world
     */
    geometry_msgs::msg::TransformStamped frame3_static_transform_stamped_;

    /**
     * @brief TransformStamped message for camera4 detected by world
     */
    geometry_msgs::msg::TransformStamped frame4_static_transform_stamped_;

    /**
     * @brief TransformStamped message for camera5 detected by world
     */
    geometry_msgs::msg::TransformStamped frame5_static_transform_stamped_;
};


