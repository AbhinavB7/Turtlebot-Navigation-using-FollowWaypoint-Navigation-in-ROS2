#include "navigate.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>
#include <algorithm>
#include <cctype>


// Function to convert a string to uppercase
std::string toUpper(const std::string& str) {
    std::string upperStr = str;
    std::transform(upperStr.begin(), upperStr.end(), upperStr.begin(),
                   [](unsigned char c){ return std::toupper(c); });
    return upperStr;
}


// function to listen to a transform between part_frame1 and map frame
void Mazenavigation::Turtlebot::part1_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // declare variables
    geometry_msgs::msg::TransformStamped t_stamped1;
    geometry_msgs::msg::Pose pose_out1;
    
    // try to get transform between source and target frame
    try
    {
        t_stamped1 = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(500));
    }
    
    // print error if transform is not found
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // store the position and orientation of the transform
    part1_pose_.push_back(t_stamped1.transform.translation.x);
    part1_pose_.push_back(t_stamped1.transform.translation.y);

    // RCLCPP_INFO_STREAM(this->get_logger(), " Part1 blue battery detected at " << part1_pose_[0]);
}



// function to listen to a transform between part_frame2 and map frame
void Mazenavigation::Turtlebot::part2_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // declare variables
    geometry_msgs::msg::TransformStamped t_stamped2;
    geometry_msgs::msg::Pose pose_out2;
    
    // try to get transform between source and target frame
    try
    {
        t_stamped2 = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(500));
    }
    
    // print error if transform is not found
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // store the position and orientation of the transform
    part2_pose_.push_back(t_stamped2.transform.translation.x);
    part2_pose_.push_back(t_stamped2.transform.translation.y);
    
    // RCLCPP_INFO_STREAM(this->get_logger(), " Part2 orange battery detected at " << part2_pose_[0]);
}



// function to listen to a transform between part_frame3 and map frame
void Mazenavigation::Turtlebot::part3_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // declare variables
    geometry_msgs::msg::TransformStamped t_stamped3;
    geometry_msgs::msg::Pose pose_out3;
    
    // try to get transform between source and target frame
    try
    {
        t_stamped3 = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(500));
    }
    
    // print error if transform is not found
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // store the position and orientation of the transform
    part3_pose_.push_back(t_stamped3.transform.translation.x);
    part3_pose_.push_back(t_stamped3.transform.translation.y);
    
    // RCLCPP_INFO_STREAM(this->get_logger(), " Part3 purple battery detected at " << part3_pose_[0]);
}



// function to listen to a transform between part_frame4 and map frame
void Mazenavigation::Turtlebot::part4_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // declare variables
    geometry_msgs::msg::TransformStamped t_stamped4;
    geometry_msgs::msg::Pose pose_out4;
    
    // try to get transform between source and target frame
    try
    {
        t_stamped4 = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(500));
    }
    
    // print error if transform is not found
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // store the position and orientation of the transform
    part4_pose_.push_back(t_stamped4.transform.translation.x);
    part4_pose_.push_back(t_stamped4.transform.translation.y);
    
    // RCLCPP_INFO_STREAM(this->get_logger(), " Part4 green battery detected at " << part4_pose_[0]);
}



// function to listen to a transform between part_frame5 and map frame
void Mazenavigation::Turtlebot::part5_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // declare variables
    geometry_msgs::msg::TransformStamped t_stamped5;
    geometry_msgs::msg::Pose pose_out5;
    
    // try to get transform between source and target frame
    try
    {
        t_stamped5 = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(500));
    }
    
    // print error if transform is not found
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // store the position and orientation of the transform
    part5_pose_.push_back(t_stamped5.transform.translation.x);
    part5_pose_.push_back(t_stamped5.transform.translation.y);
    
    // RCLCPP_INFO_STREAM(this->get_logger(), " Part5 red battery detected at " << part5_pose_[0]);

    // Store the x position and Y position of the detected battery
    if (check1_ == 0) {
      X_position_.push_back(part1_pose_[0]);
      X_position_.push_back(part2_pose_[0]);
      X_position_.push_back(part3_pose_[0]);
      X_position_.push_back(part4_pose_[0]);
      X_position_.push_back(part5_pose_[0]);
      Y_position_.push_back(part1_pose_[1]);
      Y_position_.push_back(part2_pose_[1]);
      Y_position_.push_back(part3_pose_[1]);
      Y_position_.push_back(part4_pose_[1]);
      Y_position_.push_back(part5_pose_[1]);
      check1_ = check1_+1;
    }

    // calling send_goal() once
    if (check2_ == 0) {
        send_goal();
        check2_ = check2_+1;
    }
    
}


// Subscription to mage/camera1/image topic
void Mazenavigation::Turtlebot::camera1_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
   // loop to store the part color 
   for (const auto& part_pose : msg->part_poses) {
        part1_color_ = part_pose.part.color;
        if (part1_color_ >=0){
          Camera1subscription_.reset();
        }
    }
}


// Subscription to mage/camera2/image topic
void Mazenavigation::Turtlebot::camera2_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
   // loop to store the part color 
   for (const auto& part_pose : msg->part_poses) {
        part2_color_ = part_pose.part.color;
        if (part2_color_ >=0){
          Camera2subscription_.reset();
        }
    }
}


// Subscription to mage/camera3/image topic
void Mazenavigation::Turtlebot::camera3_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
   // loop to store the part color 
   for (const auto& part_pose : msg->part_poses) {
        part3_color_ = part_pose.part.color;
        if (part3_color_ >=0){
          Camera3subscription_.reset();
        }
    }
}


// Subscription to mage/camera4/image topic
void Mazenavigation::Turtlebot::camera4_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
   // loop to store the part color 
   for (const auto& part_pose : msg->part_poses) {
        part4_color_ = part_pose.part.color;
        if (part4_color_ >=0){
          Camera4subscription_.reset();
        }
    }
}


// Subscription to mage/camera5/image topic
void Mazenavigation::Turtlebot::camera5_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
   // loop to store the part color 
   for (const auto& part_pose : msg->part_poses) {
        part5_color_ = part_pose.part.color;
        if (part5_color_ >=0){
          Camera5subscription_.reset();
        }
    }


}


// function to listen the transformation between map and part_frame1
void Mazenavigation::Turtlebot::part1_listen_timer_cb()
{
    part1_listen_transform("map", "part_frame1");
}


// function to listen the transformation between map and part_frame2
void Mazenavigation::Turtlebot::part2_listen_timer_cb()
{
    part2_listen_transform("map", "part_frame2");
}


// function to listen the transformation between map and part_frame3
void Mazenavigation::Turtlebot::part3_listen_timer_cb()
{
    part3_listen_transform("map", "part_frame3");
}


// function to listen the transformation between map and part_frame4
void Mazenavigation::Turtlebot::part4_listen_timer_cb()
{
    part4_listen_transform("map", "part_frame4");
}


// function to listen the transformation between map and part_frame5
void Mazenavigation::Turtlebot::part5_listen_timer_cb()
{
    part5_listen_transform("map", "part_frame5");
}


// fucntion to store the marker id of the ArucoMarkers
void Mazenavigation::Turtlebot::turtlebot_aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    auto marker_id = msg->marker_ids[0];
    marker_value_=marker_id;
    RCLCPP_INFO_STREAM(this->get_logger(), "Marker id is " << marker_value_);
    if (marker_value_ == 0 or 1){
      Arucosubscription_.reset();
    }

    // string objects for aruco_marker parameter retrieval
    std::string aruco_marker_param1_;
    std::string aruco_marker_param2_;
    std::string aruco_marker_param3_;
    std::string aruco_marker_param4_;
    std::string aruco_marker_param5_;

    // parameter retreival
    if (marker_value_ == 0){
      aruco_marker_param1_ = this->get_parameter("aruco_0.wp1.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param1_));
      aruco_marker_param2_ = this->get_parameter("aruco_0.wp2.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param2_));
      aruco_marker_param3_ = this->get_parameter("aruco_0.wp3.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param3_));
      aruco_marker_param4_ = this->get_parameter("aruco_0.wp4.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param4_));
      aruco_marker_param5_ = this->get_parameter("aruco_0.wp5.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param5_));
    }
    else if (marker_value_ == 1){
      aruco_marker_param1_ = this->get_parameter("aruco_1.wp1.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param1_));
      aruco_marker_param2_ = this->get_parameter("aruco_1.wp2.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param2_));
      aruco_marker_param3_ = this->get_parameter("aruco_1.wp3.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param3_));
      aruco_marker_param4_ = this->get_parameter("aruco_1.wp4.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param4_));
      aruco_marker_param5_ = this->get_parameter("aruco_1.wp5.color").as_string();
      nav_order_.push_back(toUpper(aruco_marker_param5_));
    }

    // printing the navigation order for reference
    RCLCPP_INFO_STREAM(this->get_logger(), " The first waypoint is " << aruco_marker_param1_);
    RCLCPP_INFO_STREAM(this->get_logger(), " The second waypoint is " << aruco_marker_param2_);
    RCLCPP_INFO_STREAM(this->get_logger(), " The third waypoint is " << aruco_marker_param3_);
    RCLCPP_INFO_STREAM(this->get_logger(), " The fourth waypoint is " << aruco_marker_param4_);
    RCLCPP_INFO_STREAM(this->get_logger(), " The fifth waypoint is " << aruco_marker_param5_);

    // retreiving the parameter values of part color
    for(int i =0; i<5; i++){
      if(nav_order_[i] == std::string("RED")){
        nav_order_param_.push_back(mage_msgs::msg::Part::RED);
        continue;
      }
      else if(nav_order_[i] == std::string("GREEN")){
        nav_order_param_.push_back(mage_msgs::msg::Part::GREEN);
        continue;
    }
      else if(nav_order_[i] == std::string("BLUE")){
        nav_order_param_.push_back(mage_msgs::msg::Part::BLUE);
        continue;
    }
      else if(nav_order_[i] == std::string("ORANGE")){
        nav_order_param_.push_back(mage_msgs::msg::Part::ORANGE);
        continue;
    }
      else if(nav_order_[i] == std::string("PURPLE")){
        nav_order_param_.push_back(mage_msgs::msg::Part::PURPLE);
        continue;
    }
 }
}

// funtion defination of set_initial_pose()
void Mazenavigation::Turtlebot::set_initial_pose() {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header.frame_id = "map";
    message.pose.pose.position.x = 1.0026042461395264;
    message.pose.pose.position.y = -1.5079177618026733;
    message.pose.pose.orientation.x = 0.0;
    message.pose.pose.orientation.y = 0.0;
    message.pose.pose.orientation.z = -0.698616514959973;
    message.pose.pose.orientation.w = 0.7154963067865423;
    RCLCPP_INFO_STREAM(this->get_logger(), " initilazing ");
    initial_pose_pub_->publish(message);
}


// function defination of send_goal()
void Mazenavigation::Turtlebot::send_goal() {

  if (!this->client_->wait_for_action_server(std::chrono::seconds(50))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
  }

  // storing the part colors in a vector
  if (check3_ == 0){
    part_color_.push_back(part1_color_);
    part_color_.push_back(part2_color_);
    part_color_.push_back(part3_color_);
    part_color_.push_back(part4_color_);
    part_color_.push_back(part5_color_);
    check3_ = check3_+1;
  }

  auto follow_waypoints_msg = Followwaypoints::Goal();

  geometry_msgs::msg::PoseStamped waypoint;

  // deciding the goal order
  for (size_t i{0}; i < nav_order_.size(); ++i) {
    for (size_t j{0}; j < nav_order_.size(); ++j) {
      if (nav_order_param_.at(i) == part_color_.at(j)) {
        waypoint.header.frame_id = "map";
        waypoint.header.stamp = this->get_clock()->now();
        waypoint.pose.position.x = X_position_.at(j);
        waypoint.pose.position.y = Y_position_.at(j);
        RCLCPP_INFO(this->get_logger(), "Loop executed");
        waypoint.pose.orientation.z = 0.0;
        waypoint.pose.orientation.w = 1.0;

        // Now push the waypoint into the poses vector
        follow_waypoints_msg.poses.push_back(waypoint);
        std::this_thread::sleep_for(std::chrono::seconds(5));
      }
    }
  }

  auto send_goal_options = rclcpp_action::Client<Followwaypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&Turtlebot::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback = std::bind(&Turtlebot::result_callback, this, std::placeholders::_1);

  // sending the goal
  this->client_->async_send_goal(follow_waypoints_msg, send_goal_options);
}

//===============================================
void Mazenavigation::Turtlebot::goal_response_callback(
    std::shared_future<Followwaypointsgoalhandle::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Waypoint was rejected by server");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Waypoint accepted by server, waiting for result");
  }
}

//===============================================
void Mazenavigation::Turtlebot::result_callback(
    const Followwaypointsgoalhandle::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Waypoint was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Waypoint was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}



// main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mazenavigation::Turtlebot>("Turtlebot_navigation");
  rclcpp::spin(node);
  rclcpp::shutdown();
}



