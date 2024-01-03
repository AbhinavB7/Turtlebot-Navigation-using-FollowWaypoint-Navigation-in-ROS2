/**
 * @file navigate.hpp
 * @brief Header file for the Turtlebot Movement
 * 
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include "mage_msgs/msg/part.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


// namespace Mazenavigation 
namespace Mazenavigation {

/**
 * @class Turtlebot
 * @brief A class that represents a Turtlebot node in ROS2
 */
class Turtlebot : public rclcpp::Node {
 /**
   * @brief Construct a new Turtlebot object
   * 
   * @param node_name Name of the node
   */
 public:

  // creating alias
  using Followwaypoints = nav2_msgs::action::FollowWaypoints;
  using Followwaypointsgoalhandle = rclcpp_action::ClientGoalHandle<Followwaypoints>;

  Turtlebot(std::string node_name) : Node(node_name) {

    // initialize the client
    client_ = rclcpp_action::create_client<Followwaypoints>(this, "follow_waypoints");
    // initialize the publisher
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    // set the initial pose for navigation
    set_initial_pose();
    // pause for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // Subscription to the "aruco_markers" topic

    // Parameter declaration for aruco_id 0
    this->declare_parameter("aruco_0.wp1.type", "default_type1");
    this->declare_parameter("aruco_0.wp1.color", "default_color1");
    this->declare_parameter("aruco_0.wp2.type", "default_type2");
    this->declare_parameter("aruco_0.wp2.color", "default_color2");
    this->declare_parameter("aruco_0.wp3.type", "default_type3");
    this->declare_parameter("aruco_0.wp3.color", "default_color3");
    this->declare_parameter("aruco_0.wp4.type", "default_type4");
    this->declare_parameter("aruco_0.wp4.color", "default_color4");
    this->declare_parameter("aruco_0.wp5.type", "default_type5");
    this->declare_parameter("aruco_0.wp5.color", "default_color5");

    // Parameter declaration for aruco_id 1
    this->declare_parameter("aruco_1.wp1.type", "default_type6");
    this->declare_parameter("aruco_1.wp1.color", "default_color6");
    this->declare_parameter("aruco_1.wp2.type", "default_type7");
    this->declare_parameter("aruco_1.wp2.color", "default_color7");
    this->declare_parameter("aruco_1.wp3.type", "default_type8");
    this->declare_parameter("aruco_1.wp3.color", "default_color8");
    this->declare_parameter("aruco_1.wp4.type", "default_type9");
    this->declare_parameter("aruco_1.wp4.color", "default_color9");
    this->declare_parameter("aruco_1.wp5.type", "default_type10");
    this->declare_parameter("aruco_1.wp5.color", "default_color10"); 


      // load a buffer of transforms
      tf_buffer_ =
          std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ =
          std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // timer to listen to the transform of part1
      part1_listen_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Turtlebot::part1_listen_timer_cb, this));

      // timer to listen to the transform of part2
      part2_listen_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Turtlebot::part2_listen_timer_cb, this));

      // timer to listen to the transform of part3
      part3_listen_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Turtlebot::part3_listen_timer_cb, this));

      // timer to listen to the transform of part4
      part4_listen_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Turtlebot::part4_listen_timer_cb, this));

      // timer to listen to the transform of part5
      part5_listen_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Turtlebot::part5_listen_timer_cb, this));

      // Define the Quality of Service profile for the subscriptions
      auto qos_profile = rclcpp::SensorDataQoS();

      // Create a subscription to the "/mage/camera1/image" topic
      Camera1subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera1/image", qos_profile , std::bind(&Turtlebot::camera1_sub_cb, this, std::placeholders::_1));

      // Create a subscription to the "/mage/camera2/image" topic
      Camera2subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera2/image", qos_profile , std::bind(&Turtlebot::camera2_sub_cb, this, std::placeholders::_1));

      // Create a subscription to the "/mage/camera3/image" topic
      Camera3subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera3/image", qos_profile , std::bind(&Turtlebot::camera3_sub_cb, this, std::placeholders::_1));

      // Create a subscription to the "/mage/camera4/image" topic
      Camera4subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera4/image", qos_profile , std::bind(&Turtlebot::camera4_sub_cb, this, std::placeholders::_1));

      // Create a subscription to the "/mage/camera5/image" topic
      Camera5subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera5/image", qos_profile , std::bind(&Turtlebot::camera5_sub_cb, this, std::placeholders::_1));

      // Create a subscription to the aruco_marker topic
      Arucosubscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&Turtlebot::turtlebot_aruco_sub_cb, this, std::placeholders::_1));

  }

  private:

      /**
     * @brief Publisher to the topic /initialpose
     *
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    /**
     * @brief Action client for the action server follow_waypoints
     *
     */
    rclcpp_action::Client<Followwaypoints>::SharedPtr client_;

    /**
     * @brief callback function to check status of goal
     * 
     * @param future 
     */
    void goal_response_callback(std::shared_future<Followwaypointsgoalhandle::SharedPtr> future);
    /**
     * @brief callback function to check the final status of goal
     * 
     * @param result 
     */
    void result_callback(const Followwaypointsgoalhandle::WrappedResult& result);
    /**
     * @brief Method to set the initial pose
     * 
     */
    void set_initial_pose();
    /**
     * @brief Method to build and send a waypoint using the client
     * 
     */
    void send_goal();

    /**
     * @brief Buffer to store transform data for easy lookup
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /**
     * @brief Transform listener to listen for transformation updates
     */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

   /**
    * @brief Timer object to listen the transformation of part1
    */
   rclcpp::TimerBase::SharedPtr part1_listen_timer_;

   /**
    * @brief Timer object to listen the transformation of part2
    */
   rclcpp::TimerBase::SharedPtr part2_listen_timer_;

   /**
    * @brief Timer object to listen the transformation of part3
    */
   rclcpp::TimerBase::SharedPtr part3_listen_timer_;

   /**
    * @brief Timer object to listen the transformation of part4
    */
   rclcpp::TimerBase::SharedPtr part4_listen_timer_;

   /**
    * @brief Timer object to listen the transformation of part5
    */
   rclcpp::TimerBase::SharedPtr part5_listen_timer_;
    
    /**
     * @brief method to listen the transform of part1
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part1_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief method to listen the transform of part2
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part2_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief method to listen the transform of part3
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part3_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief method to listen the transform of part4
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part4_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief method to listen the transform of part5
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part5_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Timer callback to listen to the transform of part1
     *
     */
    void part1_listen_timer_cb();

    /**
     * @brief Timer callback to listen to the transform of part2
     *
     */
    void part2_listen_timer_cb();

    /**
     * @brief Timer callback to listen to the transform of part3
     *
     */
    void part3_listen_timer_cb();

    /**
     * @brief Timer callback to listen to the transform of part4
     *
     */
    void part4_listen_timer_cb();

    /**
     * @brief Timer callback to listen to the transform of part5
     *
     */
    void part5_listen_timer_cb();

    /**
     * @brief Subscription object to aruco_markers topic
     */    
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr Arucosubscription_;

    /**
     * @brief Timer callback function for the aruco_markers topic
     *
     */
    void turtlebot_aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

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


    // declare variables for battery color
    int part1_color_;

    int part2_color_;

    int part3_color_;

    int part4_color_;

    int part5_color_;

    // declare vector to store part colors
    std::vector<int> part_color_;

    // declare vector to store x-positions of parts
    std::vector<double> X_position_;

    // declare vector to store y-positions of parts
    std::vector<double> Y_position_;

    // declaration of counter variables
    int check1_{0};
    int check2_{0};
    int check3_{0};

    // declaration of vectors to store pose
    std::vector<double> part1_pose_;
    std::vector<double> part2_pose_;
    std::vector<double> part3_pose_;
    std::vector<double> part4_pose_;
    std::vector<double> part5_pose_;

    // declaration of variable to store aruco marker value
    int marker_value_{};

    // Declare a vector of type string to store the navigation order
    std::vector<std::string> nav_order_;

    // Declare a vector of type int to store the navigation parameter order
    std::vector<int> nav_order_param_;

  };  // class Turtlebot
}  // namespace Mazenavigation