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

#include "incremental_arm_mover.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf_conversions/tf_kdl.h>

IncrementalArmMover::IncrementalArmMover(ros::NodeHandle pnh)
{
  pnh.param<std::string>( "tracked_frame", target_frame_id_, "hydra_right_pivot");
  pnh.param<std::string>( "root_frame", root_frame_id_, "base_link");
  pnh.param<int>( "deadman_button", deadman_button_, 0);
  pnh.param<double>( "update_freq", update_freq_, 0.1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&IncrementalArmMover::joyCb, this, _1));
  command_pub_ = nh_.advertise<geometry_msgs::Twist>("command", 1);

  last_deadman_state_ = false;
}

IncrementalArmMover::~IncrementalArmMover()
{
}


void IncrementalArmMover::joyCb(sensor_msgs::JoyConstPtr joy_msg)
{
  if (joy_msg->buttons.size() <= deadman_button_)
  {
    ROS_ERROR_ONCE("Button index for deadman switch is out of bounds!");
    return;
  }

  if ( (ros::Time::now() - last_update_time_) < ros::Duration(update_freq_))
  {
    return;
  }

  ros::Time now(ros::Time::now());

  /* Need transform */
  if (!tf_.waitForTransform(target_frame_id_, root_frame_id_, now, ros::Duration(0.1)))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "IncrementalArmMover: Unable to transform " 
                              << target_frame_id_ << " to " << root_frame_id_ << ".");
    return;
  }

  bool deadman_state = joy_msg->buttons.at(deadman_button_);    

  tf::StampedTransform transform;
  geometry_msgs::PoseStamped stamped;
  stamped.pose.orientation.w = 1.0;
  stamped.header.stamp = now;
  stamped.header.frame_id = target_frame_id_;
  tf_.transformPose(root_frame_id_, stamped, stamped);
  tf::Stamped<tf::Pose> tf_pose_stamped;
  tf::poseStampedMsgToTF(stamped, tf_pose_stamped);
  KDL::Frame pose;
  tf::poseTFToKDL(tf_pose_stamped,pose);

  if (deadman_state && last_deadman_state_)
  {
    last_update_time_ = now;
    KDL::Twist twist = KDL::diff(last_pose_, pose, 1.0);
    geometry_msgs::Twist t;
    t.linear.x = twist(0);
    t.linear.y = twist(1);
    t.linear.z = twist(2);
    t.angular.x = twist(3);
    t.angular.y = twist(4);
    t.angular.z = twist(5);
    command_pub_.publish(t);
  }

  last_deadman_state_ = deadman_state;
  last_pose_ = pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_mover");
  ros::NodeHandle pnh("~");

  IncrementalArmMover arm_mover(pnh);

  ros::spin();
  return 0;
}
