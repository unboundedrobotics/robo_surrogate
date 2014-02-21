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

#ifndef ROBO_SURROGATE_ARM_PRE_MOVER_H_
#define ROBO_SURROGATE_ARM_PRE_MOVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
//#include <geometry_msgs/PoseStamped.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <kdl/frames.hpp>
#include <vector>

class ArmPreMover
{
public:
  ArmPreMover(ros::NodeHandle pnh);
  virtual ~ArmPreMover();

private:
  void joyCb(sensor_msgs::JoyConstPtr joy_msg);
  void jointStateCb(sensor_msgs::JointStateConstPtr msg);

  ros::Time last_update_time_;
  ros::Time last_joy_update_time_;

  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher target_joint_states_pub_;
  ros::Publisher follow_trajectory_goal_pub_;

  std::vector<double> current_joint_positions_;
  sensor_msgs::JointState target_joint_states_;
  
  KDL::Chain kdl_chain_;
  //boost::shared_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  //boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  boost::shared_ptr<KDL::ChainIkSolverVel_wdls> solver_;
  KDL::Jacobian jacobian_;
  KDL::JntArray jnt_pos_;
  KDL::JntArray jnt_vel_;

  /// 
  double update_period_;

  /// index of button to used for moving gripper/arm
  int move_button_;

  /// index of button used for executing move
  int execute_button_;
 
  bool initialized_;
  bool have_current_joint_state_;
};

#endif // ROBO_SURROGATE_ARM_PRE_MOVER_H_
