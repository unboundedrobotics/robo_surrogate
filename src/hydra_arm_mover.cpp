/*********************************************************************
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  Copyright (c) 2014, Unbounded Robotics, Inc.
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
  pnh.param<double>( "update_period", update_period_, 0.1);
  pnh.param<int>( "deadman_button", deadman_button_, 11); // right top trigger on hydra
  pnh.param<int>( "gripper_open_button", gripper_open_button_, 12); // right hydra button 4
  pnh.param<int>( "gripper_close_button", gripper_close_button_, 14); // right hydra button 2

  pnh.param<double>( "arm_linear_scale", arm_linear_scale_, 0.6);
  pnh.param<double>( "arm_angular_scale", arm_angular_scale_, 0.3);

  pnh.param<double>("gripper_open_pos", gripper_open_pos_, 0.09);
  pnh.param<double>("gripper_close_pos", gripper_close_pos_, 0.0);
  pnh.param<double>("gripper_max_effort", gripper_max_effort_, 28.0);

  // Gripper is neither opened nor closed to start with
  gripper_opened_ = false;
  gripper_closed_ = false;

  // We have not recv'ed a joystick pose from TF just yet
  have_last_pose_ = false;

  // Deadman has not been pushed
  last_deadman_counter_ = 0;
  
  /* Gripper action setup */
  gripper_client_.reset( new GripperClient("gripper_controller/gripper_action", true) );
  if (!gripper_client_->waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("gripper_controller/gripper_action may not be connected. Gripper may not move.");
  }  

  ros::NodeHandle gnh;
  joy_sub_ = gnh.subscribe<sensor_msgs::Joy>("hydra_joy", 1, boost::bind(&HydraArmMover::joyCb, this, _1));
  command_pub_ = gnh.advertise<geometry_msgs::Twist>("arm_controller/cartesian_twist/command", 1);
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


  if (!tf_.waitForTransform(target_frame_id_, root_frame_id_, ros::Time::now(), ros::Duration(0.5)))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "HydraArmMover: Unable to transform " 
                              << target_frame_id_ << " to " << root_frame_id_ << ".");
    return;
  }

  ros::Time now(ros::Time::now());

  KDL::Frame pose;
  tf::StampedTransform transform;
  geometry_msgs::PoseStamped stamped;
  stamped.pose.orientation.w = 1.0;
  stamped.header.stamp = now - ros::Duration(0.1);
  stamped.header.frame_id = target_frame_id_;

  try
  {
    tf_.transformPose(root_frame_id_, stamped, stamped);
  } 
  catch (tf2::ExtrapolationException except)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "HydraArmMover: transform exception " << except.what());
    // try again, this time adding a little bit more delay
    stamped.header.stamp = now - ros::Duration(0.25);
    tf_.transformPose(root_frame_id_, stamped, stamped);
  }

  tf::Stamped<tf::Pose> tf_pose_stamped;
  tf::poseStampedMsgToTF(stamped, tf_pose_stamped);
  tf::poseTFToKDL(tf_pose_stamped,pose);


  /* If deadman is pressed or just released, send Twist command
   * While deadman is released, don't sending messages which will cause 
   *   cartesian twist controller to timeout and release arm
   *
   * If we do not have a pose message from last cycle to compare to, send zero Twist
   * If deadman was just released, send zero Twist command forces arm to stop quickly
   * Otherwise, calculate difference in current and last hydra pose and send it as twist
   */
  bool deadman = joy_msg->buttons[deadman_button_];
  KDL::Twist twist;
  for (int ii=0;ii<6;++ii) twist(ii) = 0.0;

  if (deadman && have_last_pose_)
  {
    twist = KDL::diff(last_pose_, pose, 1.0);
    double dt = (now - last_update_time_).toSec();
    if (dt<=1e-3) 
      dt=1e-3;

    double linear_scale = arm_linear_scale_ / dt;
    twist(0) = twist(0) * linear_scale;
    twist(1) = twist(1) * linear_scale;
    twist(2) = twist(2) * linear_scale;
    double angular_scale = arm_angular_scale_ / dt;
    twist(3) = twist(3) * angular_scale;
    twist(4) = twist(4) * angular_scale;
    twist(5) = twist(5) * angular_scale;
  }

  double filter_coeff = 0.5;
  for (int ii=0;ii<6;++ii)
  {
    twist_filtered_(ii) += filter_coeff * (twist(ii) - twist_filtered_(ii));
  }

  if (last_deadman_counter_ == 0)
  {
    for (int ii=0;ii<6;++ii) twist_filtered_(ii) = 0;
  }

  if (deadman || (last_deadman_counter_>0))
  {
    geometry_msgs::Twist cmd; 
    cmd.linear.x  = twist_filtered_(0);
    cmd.linear.y  = twist_filtered_(1);
    cmd.linear.z  = twist_filtered_(2);
    cmd.angular.x = twist_filtered_(3);
    cmd.angular.y = twist_filtered_(4);
    cmd.angular.z = twist_filtered_(5);
    command_pub_.publish(cmd);
  }


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


  last_update_time_ = ros::Time::now();
  last_pose_ = pose;
  have_last_pose_ = true;

  if (deadman)
  {
    last_deadman_counter_ = 10;
  }
  else if (last_deadman_counter_ > 0)
  {
    --last_deadman_counter_;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hydra_arm_mover");
  ros::NodeHandle pnh("~");

  HydraArmMover hydra(pnh);

  ros::spin();

  return 0;
}
