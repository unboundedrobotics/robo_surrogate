/*********************************************************************
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "hydra_arm_mover.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <robo_surrogate/ArmMove.h>
#include <tf_conversions/tf_kdl.h>

HydraArmMover::HydraArmMover(ros::NodeHandle pnh)
{
  pnh.param<std::string>( "tracked_frame", target_frame_id_, "hydra_right_pivot");
  pnh.param<std::string>( "root_frame", root_frame_id_, "base_link");
  pnh.param<double>( "update_period", update_period_, 0.05);
  nh_.param<int>( "deadman_button", deadman_button_, 11); // right top trigger on hydra
  nh_.param<int>( "move_button", move_button_, 9); // right bottom trigger on hydra
  nh_.param<int>( "gripper_open_button", gripper_open_button_, 12); // right hydra button 4
  nh_.param<int>( "gripper_close_button", gripper_close_button_, 14); // right hydra button 2

  nh_.param("gripper_open_pos", gripper_open_pos_, 0.09);
  nh_.param("gripper_close_pos", gripper_close_pos_, 0.0);
  nh_.param("gripper_max_effort", gripper_max_effort_, 28.0);

  // Gripper is neither opened nor closed to start with
  gripper_opened_ = false;
  gripper_closed_ = false;
  
  /* Gripper action setup */
  gripper_client_.reset( new GripperClient("gripper_controller/gripper_action", true) );
  if (!gripper_client_->waitForServer(ros::Duration(2.0)))
  {
      ROS_ERROR("gripper_controller/gripper_action may not be connected. Gripper may not move.");
  }  

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("hydra_joy", 1, boost::bind(&HydraArmMover::joyCb, this, _1));
  command_pub_ = nh_.advertise<robo_surrogate::ArmMove>("move_command", 1);
}

HydraArmMover::~HydraArmMover()
{
}


void HydraArmMover::joyCb(sensor_msgs::JoyConstPtr joy_msg)
{
  ROS_ERROR_ONCE("JoyCB");

  if (ros::Time::now() - last_update_time_ < ros::Duration(update_period_))
  {
    return;
  }
  
  if (joy_msg->buttons.size() <= deadman_button_)
  {
    ROS_ERROR_ONCE("Deadman button index is out of bounds!");
    return;
  }
  if (joy_msg->buttons.size() <= move_button_)
  {
    ROS_ERROR_ONCE("Move button index is out of bounds!");
    return;
  }
  if (joy_msg->buttons.size() <= gripper_open_button_)
  {
    ROS_ERROR_ONCE("Gripper op button index is out of bounds!");
    return;
  }
  if (joy_msg->buttons.size() <= gripper_close_button_)
  {
    ROS_ERROR_ONCE("Gripper op button index is out of bounds!");
    return;
  }


  /* Need transform */
  if (!tf_.waitForTransform(target_frame_id_, root_frame_id_, ros::Time::now(), ros::Duration(0.5)))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "HydraArmMover: Unable to transform " 
                              << target_frame_id_ << " to " << root_frame_id_ << ".");
    return;
  }

  ros::Time now = ros::Time::now(); 
  tf::StampedTransform transform;
  geometry_msgs::PoseStamped stamped;
  stamped.pose.orientation.w = 1.0;
  stamped.header.stamp = now - ros::Duration(0.2);
  stamped.header.frame_id = target_frame_id_;
  tf_.transformPose(root_frame_id_, stamped, stamped);
  tf::Stamped<tf::Pose> tf_pose_stamped;
  tf::poseStampedMsgToTF(stamped, tf_pose_stamped);
  KDL::Frame pose;
  tf::poseTFToKDL(tf_pose_stamped,pose);

  KDL::Twist twist = KDL::diff(last_pose_, pose, 1.0);

  robo_surrogate::ArmMove move;
  move.deadman = joy_msg->buttons.at(deadman_button_);
  move.move = false; //joy_msg->buttons.at(move_button_);
  geometry_msgs::Twist &t(move.twist);
  t.linear.x = twist(0);
  t.linear.y = twist(1);
  t.linear.z = twist(2);
  t.angular.x = twist(3);
  t.angular.y = twist(4);
  t.angular.z = twist(5);
  command_pub_.publish(move);    

  if (joy_msg->buttons.at(gripper_open_button_) && !gripper_opened_)
  {
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gripper_open_pos_;
    goal.command.max_effort = gripper_max_effort_;
    gripper_client_->sendGoal(goal);
    gripper_opened_ = true;
    gripper_closed_ = false;
  }
  else if (joy_msg->buttons.at(gripper_close_button_) && !gripper_closed_)
  {
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gripper_close_pos_;
    goal.command.max_effort = gripper_max_effort_;
    gripper_client_->sendGoal(goal);
    gripper_closed_ = true;
    gripper_opened_ = false;
  }
  
  last_update_time_ = now;
  last_pose_ = pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hydra_arm_mover");
  ros::NodeHandle pnh("~");

  HydraArmMover hydra(pnh);

  ros::spin();

  return 0;
}
