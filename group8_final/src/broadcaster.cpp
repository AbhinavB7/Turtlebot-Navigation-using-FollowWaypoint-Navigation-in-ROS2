#include <broadcaster.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

// callback function for the subscription to the clock
void Broadcaster::sub_clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
   // get the current time
   current_time_ = msg->clock;
}


// callback function for the subscription to /mage/camera1/image topic
void Broadcaster::camera1_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // get the pose of the camera1
  auto poses = msg->sensor_pose;
  frame1_static_transform_stamped_.transform.translation.x = poses.position.x;
  frame1_static_transform_stamped_.transform.translation.y = poses.position.y;
  frame1_static_transform_stamped_.transform.translation.z = poses.position.z;
  frame1_static_transform_stamped_.transform.rotation.x = poses.orientation.x;
  frame1_static_transform_stamped_.transform.rotation.y = poses.orientation.y;
  frame1_static_transform_stamped_.transform.rotation.z = poses.orientation.z;
  frame1_static_transform_stamped_.transform.rotation.w = poses.orientation.w;

  // loop for each part
  for (const auto& part_pose : msg->part_poses) {
    // get the pose of the part1
    auto pose = part_pose.pose;
    camera1_static_transform_stamped_.transform.translation.x = pose.position.x;
    camera1_static_transform_stamped_.transform.translation.y = pose.position.y;
    camera1_static_transform_stamped_.transform.translation.z = pose.position.z;
    camera1_static_transform_stamped_.transform.rotation.x = pose.orientation.x;
    camera1_static_transform_stamped_.transform.rotation.y = pose.orientation.y;
    camera1_static_transform_stamped_.transform.rotation.z = pose.orientation.z;
    camera1_static_transform_stamped_.transform.rotation.w = pose.orientation.w;
    if (pose.position.x >= 0){
      Camera1subscription_.reset();
    }
  }
}

// callback function for the subscription to /mage/camera1/image topic
void Broadcaster::camera2_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

// get the pose of the camera2
  auto poses = msg->sensor_pose;
  frame2_static_transform_stamped_.transform.translation.x = poses.position.x;
  frame2_static_transform_stamped_.transform.translation.y = poses.position.y;
  frame2_static_transform_stamped_.transform.translation.z = poses.position.z;
  frame2_static_transform_stamped_.transform.rotation.x = poses.orientation.x;
  frame2_static_transform_stamped_.transform.rotation.y = poses.orientation.y;
  frame2_static_transform_stamped_.transform.rotation.z = poses.orientation.z;
  frame2_static_transform_stamped_.transform.rotation.w = poses.orientation.w;

  // loop for each part
  for (const auto& part_pose : msg->part_poses) {
    // get the pose of the part2
    auto pose = part_pose.pose;
    camera2_static_transform_stamped_.transform.translation.x = pose.position.x;
    camera2_static_transform_stamped_.transform.translation.y = pose.position.y;
    camera2_static_transform_stamped_.transform.translation.z = pose.position.z;
    camera2_static_transform_stamped_.transform.rotation.x = pose.orientation.x;
    camera2_static_transform_stamped_.transform.rotation.y = pose.orientation.y;
    camera2_static_transform_stamped_.transform.rotation.z = pose.orientation.z;
    camera2_static_transform_stamped_.transform.rotation.w = pose.orientation.w;
    if (pose.position.x >= 0){
      Camera2subscription_.reset();
    }
  }
}



// callback function for the subscription to /mage/camera1/image topic
void Broadcaster::camera3_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

// get the pose of the camera3
  auto poses = msg->sensor_pose;
  frame3_static_transform_stamped_.transform.translation.x = poses.position.x;
  frame3_static_transform_stamped_.transform.translation.y = poses.position.y;
  frame3_static_transform_stamped_.transform.translation.z = poses.position.z;
  frame3_static_transform_stamped_.transform.rotation.x = poses.orientation.x;
  frame3_static_transform_stamped_.transform.rotation.y = poses.orientation.y;
  frame3_static_transform_stamped_.transform.rotation.z = poses.orientation.z;
  frame3_static_transform_stamped_.transform.rotation.w = poses.orientation.w;

  // loop for each part
  for (const auto& part_pose : msg->part_poses) {
    // get the pose of the part3
    auto pose = part_pose.pose;
    camera3_static_transform_stamped_.transform.translation.x = pose.position.x;
    camera3_static_transform_stamped_.transform.translation.y = pose.position.y;
    camera3_static_transform_stamped_.transform.translation.z = pose.position.z;
    camera3_static_transform_stamped_.transform.rotation.x = pose.orientation.x;
    camera3_static_transform_stamped_.transform.rotation.y = pose.orientation.y;
    camera3_static_transform_stamped_.transform.rotation.z = pose.orientation.z;
    camera3_static_transform_stamped_.transform.rotation.w = pose.orientation.w;
    if (pose.position.x >= 0){
      Camera3subscription_.reset();
    }
  }

}



// callback function for the subscription to /mage/camera1/image topic
void Broadcaster::camera4_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

// get the pose of the camera4
  auto poses = msg->sensor_pose;
  frame4_static_transform_stamped_.transform.translation.x = poses.position.x;
  frame4_static_transform_stamped_.transform.translation.y = poses.position.y;
  frame4_static_transform_stamped_.transform.translation.z = poses.position.z;
  frame4_static_transform_stamped_.transform.rotation.x = poses.orientation.x;
  frame4_static_transform_stamped_.transform.rotation.y = poses.orientation.y;
  frame4_static_transform_stamped_.transform.rotation.z = poses.orientation.z;
  frame4_static_transform_stamped_.transform.rotation.w = poses.orientation.w;

  // loop for each part
  for (const auto& part_pose : msg->part_poses) {
    // get the pose of the part4
    auto pose = part_pose.pose;
    camera4_static_transform_stamped_.transform.translation.x = pose.position.x;
    camera4_static_transform_stamped_.transform.translation.y = pose.position.y;
    camera4_static_transform_stamped_.transform.translation.z = pose.position.z;
    camera4_static_transform_stamped_.transform.rotation.x = pose.orientation.x;
    camera4_static_transform_stamped_.transform.rotation.y = pose.orientation.y;
    camera4_static_transform_stamped_.transform.rotation.z = pose.orientation.z;
    camera4_static_transform_stamped_.transform.rotation.w = pose.orientation.w;
    if (pose.position.x >= 0){
      Camera4subscription_.reset();
    }
  }

}



// callback function for the subscription to /mage/camera1/image topic
void Broadcaster::camera5_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

// get the pose of the camera5
  auto poses = msg->sensor_pose;
  frame5_static_transform_stamped_.transform.translation.x = poses.position.x;
  frame5_static_transform_stamped_.transform.translation.y = poses.position.y;
  frame5_static_transform_stamped_.transform.translation.z = poses.position.z;
  frame5_static_transform_stamped_.transform.rotation.x = poses.orientation.x;
  frame5_static_transform_stamped_.transform.rotation.y = poses.orientation.y;
  frame5_static_transform_stamped_.transform.rotation.z = poses.orientation.z;
  frame5_static_transform_stamped_.transform.rotation.w = poses.orientation.w;

  // loop for each part
  for (const auto& part_pose : msg->part_poses) {
    // get the pose of the part5
    auto pose = part_pose.pose;
    camera5_static_transform_stamped_.transform.translation.x = pose.position.x;
    camera5_static_transform_stamped_.transform.translation.y = pose.position.y;
    camera5_static_transform_stamped_.transform.translation.z = pose.position.z;
    camera5_static_transform_stamped_.transform.rotation.x = pose.orientation.x;
    camera5_static_transform_stamped_.transform.rotation.y = pose.orientation.y;
    camera5_static_transform_stamped_.transform.rotation.z = pose.orientation.z;
    camera5_static_transform_stamped_.transform.rotation.w = pose.orientation.w;
    if (pose.position.x >= 0){
      Camera5subscription_.reset();
    }
  }

}


void Broadcaster::static_broadcast_timer_cb_()
{
    geometry_msgs::msg::TransformStamped static_transform_stamped_;

    // to broadcast transformation between map and camera1 frame
    frame1_static_transform_stamped_.header.stamp = current_time_;
    frame1_static_transform_stamped_.header.frame_id = "map";
    frame1_static_transform_stamped_.child_frame_id = "camera1_frame";
    tf_static_broadcaster_->sendTransform(frame1_static_transform_stamped_);

    // to broadcast transformation between map and camera2 frame
    frame2_static_transform_stamped_.header.stamp = current_time_;
    frame2_static_transform_stamped_.header.frame_id = "map";
    frame2_static_transform_stamped_.child_frame_id = "camera2_frame";
    tf_static_broadcaster_->sendTransform(frame2_static_transform_stamped_);

    // to broadcast transformation between map and camera3 frame
    frame3_static_transform_stamped_.header.stamp = current_time_;
    frame3_static_transform_stamped_.header.frame_id = "map";
    frame3_static_transform_stamped_.child_frame_id = "camera3_frame";
    tf_static_broadcaster_->sendTransform(frame3_static_transform_stamped_);

    // to broadcast transformation between map and camera4 frame
    frame4_static_transform_stamped_.header.stamp = current_time_;
    frame4_static_transform_stamped_.header.frame_id = "map";
    frame4_static_transform_stamped_.child_frame_id = "camera4_frame";
    tf_static_broadcaster_->sendTransform(frame4_static_transform_stamped_);

    // to broadcast transformation between map and camera5 frame
    frame5_static_transform_stamped_.header.stamp = current_time_;
    frame5_static_transform_stamped_.header.frame_id = "map";
    frame5_static_transform_stamped_.child_frame_id = "camera5_frame";
    tf_static_broadcaster_->sendTransform(frame5_static_transform_stamped_);

    // to broadcast transformation between camera1 frame and part1 frame
    camera1_static_transform_stamped_.header.stamp = current_time_;
    camera1_static_transform_stamped_.header.frame_id = "camera1_frame";
    camera1_static_transform_stamped_.child_frame_id = "part_frame1";
    tf_static_broadcaster_->sendTransform(camera1_static_transform_stamped_);

    // to broadcast transformation between camera2 frame and part2 frame
    camera2_static_transform_stamped_.header.stamp = current_time_;
    camera2_static_transform_stamped_.header.frame_id = "camera2_frame";
    camera2_static_transform_stamped_.child_frame_id = "part_frame2";
    tf_static_broadcaster_->sendTransform(camera2_static_transform_stamped_);

    // to broadcast transformation between camera3 frame and part3 frame
    camera3_static_transform_stamped_.header.stamp = current_time_;
    camera3_static_transform_stamped_.header.frame_id = "camera3_frame";
    camera3_static_transform_stamped_.child_frame_id = "part_frame3";
    tf_static_broadcaster_->sendTransform(camera3_static_transform_stamped_);

    // to broadcast transformation between camera4 frame and part4 frame
    camera4_static_transform_stamped_.header.stamp = current_time_;
    camera4_static_transform_stamped_.header.frame_id = "camera4_frame";
    camera4_static_transform_stamped_.child_frame_id = "part_frame4";
    tf_static_broadcaster_->sendTransform(camera4_static_transform_stamped_);

    // to broadcast transformation between camera5 frame and part5 frame
    camera5_static_transform_stamped_.header.stamp = current_time_;
    camera5_static_transform_stamped_.header.frame_id = "camera5_frame";
    camera5_static_transform_stamped_.child_frame_id = "part_frame5";
    tf_static_broadcaster_->sendTransform(camera5_static_transform_stamped_);

}




// main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Broadcaster>("broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}

















