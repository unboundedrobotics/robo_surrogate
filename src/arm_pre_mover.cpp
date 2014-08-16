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

#include "arm_pre_mover.h"
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>


ArmPreMover::ArmPreMover(ros::NodeHandle nh)
{
  initialized_ = false;
  have_current_joint_state_ = false;

  nh_ = nh;
  std::string tip_link, root_link;
  nh_.param<std::string>("root_name", root_link, "torso_lift_link");
  nh_.param<std::string>("tip_name", tip_link, "gripper_link");
  nh_.param<double>( "update_period", update_period_, 0.1);


  /* Load URDF */
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return;
  }

  /* Load the tree */
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    return;
  }

  /* Populate the Chain */
  if(!kdl_tree.getChain(root_link, tip_link, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return;
  }

  //jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  //jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  solver_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain_)  );
  //jnt_delta_.resize(kdl_chain_.getNrOfJoints());
  unsigned num_joints = kdl_chain_.getNrOfJoints();
  jacobian_.resize(num_joints);
  jnt_pos_.resize(num_joints);
  jnt_vel_.resize(num_joints);
  target_joint_states_.position.resize(num_joints);
  target_joint_states_.velocity.resize(num_joints);
  target_joint_states_.effort.resize(num_joints);
  current_joint_positions_.resize(num_joints);

  ROS_ERROR("kdl_chain_.getNrOfSegments() = %d", kdl_chain_.getNrOfSegments());
  ROS_ERROR("kdl_chain_.getNrOfJoints() = %d", kdl_chain_.getNrOfJoints());

  for (size_t ii = 0; ii < kdl_chain_.getNrOfSegments(); ++ii)
  {
    if (kdl_chain_.getSegment(ii).getJoint().getType() != KDL::Joint::None)
    {
      target_joint_states_.name.push_back(kdl_chain_.getSegment(ii).getJoint().getName());      
      ROS_ERROR("Joint %s", kdl_chain_.getSegment(ii).getJoint().getName().c_str());
    }
    else 
    {
      ROS_ERROR("None Joint %s", kdl_chain_.getSegment(ii).getJoint().getName().c_str());
    }
  }

  /* Get jacobian */
  if (target_joint_states_.name.size() != num_joints)
  {
    ROS_ERROR("Inconsistance joint count %d, %d", num_joints, int(target_joint_states_.name.size()));
    return;
  }


  move_sub_ = nh_.subscribe<robo_surrogate::ArmMove>("move_command", 1, boost::bind(&ArmPreMover::moveCb, this, _1));
  joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&ArmPreMover::jointStateCb, this, _1));
  target_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("arm_pre_mover/joint_states", 1);
  follow_trajectory_goal_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1);

  last_update_time_ = ros::Time(0.0);
  last_move_update_time_ = ros::Time(0.0);

  ROS_ERROR_ONCE("ArmPreMover Complete");
  initialized_ = true;
}

ArmPreMover::~ArmPreMover()
{
}


void ArmPreMover::jointStateCb(sensor_msgs::JointStateConstPtr msg)
{
  if (msg->position.size() != msg->name.size())
  {
    ROS_WARN_THROTTLE(1.0, "Joint states names and positions are different sizes");
    return;
  }

  // TODO put lock on this update
  
  // Move button is not held-down reset target joint states to current joint states
  for (unsigned ii=0; ii<target_joint_states_.name.size(); ++ii)
  {
    for (unsigned jj=0; jj<msg->name.size(); ++jj)
    {
      if (target_joint_states_.name[ii].compare(msg->name[jj]) == 0)
      {
        current_joint_positions_[ii] = msg->position[jj];
      }
    }
  }

  have_current_joint_state_ = true;
  ROS_ERROR_ONCE("JointStateCB (2)");
}

void ArmPreMover::moveCb(robo_surrogate::ArmMoveConstPtr move_msg)
{
  // TODO put lock on this update

  ROS_ERROR_ONCE("MoveCB");

  ros::Time now(ros::Time::now());

  if (!initialized_ || !have_current_joint_state_)
  {
    return;
  }
 
  if (!move_msg->move && !move_msg->deadman)
  {
    target_joint_states_.position = current_joint_positions_;
  }
  else // move button is pressed
  {
    // Move button is held down move target joints states using jacobian

    ROS_ERROR_ONCE("MovePressed");

    KDL::Twist twist;
    twist(0) = move_msg->twist.linear.x;
    twist(1) = move_msg->twist.linear.y;
    twist(2) = move_msg->twist.linear.z;
    twist(3) = move_msg->twist.angular.x;
    twist(4) = move_msg->twist.angular.y;
    twist(5) = move_msg->twist.angular.z;

    unsigned num_joints = target_joint_states_.position.size();
    for (unsigned ii = 0; ii < num_joints; ++ii)
    {
      jnt_pos_(ii) = target_joint_states_.position[ii];
    }

    if (solver_->CartToJnt(jnt_pos_, twist, jnt_vel_) < 0)
    {
      ROS_ERROR_THROTTLE(1.0, "KDL Cartesian Velocity Solver Failed");    
    }
    else 
    {
      // If any joint movement is faster than max velocity, scale all joint movements back      
      double max_delta = 0.0;
      for (unsigned ii = 0; ii < num_joints; ++ii)
      {
        max_delta = std::max(std::abs(jnt_vel_(ii)), max_delta);
      }

      double dt = (now - last_move_update_time_).toSec();
      double velocity_limit = 0.5;
      double delta_limit = velocity_limit * dt;
      double delta_scale = 1.0;
      if (max_delta > delta_limit)
      {
        delta_scale = delta_limit / max_delta;
        ROS_ERROR_THROTTLE(1.0, "Limiting Velocity");
      }

      for (unsigned ii = 0; ii < num_joints; ++ii)
      {
        target_joint_states_.position[ii] += jnt_vel_(ii) * delta_scale;
      }
    }
  }

  // If enough time has passed, send update
  if ( (now - last_update_time_) > ros::Duration(update_period_) )
  {
    target_joint_states_.header.stamp = now;
    target_joint_states_pub_.publish(target_joint_states_);
    ROS_ERROR_ONCE("PreMoverFirstUpdate");  

    control_msgs::FollowJointTrajectoryActionGoal action_goal;
    control_msgs::FollowJointTrajectoryGoal &goal(action_goal.goal);
    trajectory_msgs::JointTrajectory &traj(goal.trajectory);
    traj.header.stamp = now;
    traj.joint_names = target_joint_states_.name;    

    if (move_msg->deadman)
    {
      // Send joint trajectory action
      traj.points.resize(1);
      //traj.points[0].positions = current_joint_positions_;
      traj.points[0].positions = target_joint_states_.position;
      traj.points[0].time_from_start = ros::Duration(3*update_period_); // TODO calculate this based on max distance traveled?
    }
    else 
    {
      traj.points.resize(0);
      // an empty trajectory should cause trajectory follower to stop what it is doing
    }
    follow_trajectory_goal_pub_.publish(action_goal);      

    last_update_time_ = now;
  }

  ROS_ERROR_ONCE("PreMoverFirstComplete");
  last_move_update_time_ = now;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_pre_mover");
  ros::NodeHandle pnh("");

  ArmPreMover arm_mover(pnh);

  ros::spin();
  return 0;
}
