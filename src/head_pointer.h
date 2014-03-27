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

#ifndef ROBO_SURROGATE_HEAD_POINTER_H_
#define ROBO_SURROGATE_HEAD_POINTER_H_

#include <ros/ros.h>

#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/Joy.h>

#include <actionlib/client/simple_action_client.h>

class HeadPointer
{
  typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadActionClient;

public:
  HeadPointer( ros::NodeHandle pnh, std::string action_topic );
  virtual ~HeadPointer();

private:
  void joyCb( sensor_msgs::JoyConstPtr joy_msg );

  PointHeadActionClient point_head_action_client_;
  control_msgs::PointHeadGoal point_head_goal_;

  ros::Time last_update_time_;
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;

  double update_period_;
  int deadman_button_;
  int alt_deadman_button_;
};

#endif  // ROBO_SURROGATE_HEAD_POINTER_H_
