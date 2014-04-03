/*
 * follow_joint_trajectory_client.cpp
 *
 *  Created on: 06.11.2011
 *      Author: martin
 *  Modified on: 28.02.2014
 * 	Author: JakaCikac
 */

#include <katana_tutorials/follow_joint_trajectory_client.h>
#include "ros/ros.h"

namespace katana_tutorials
{

FollowJointTrajectoryClient::FollowJointTrajectoryClient() :
    traj_client_("/katana_arm_controller/follow_joint_trajectory", true), gripper_("gripper_grasp_posture_controller",true), got_joint_state_(false), spinner_(1)
{
  joint_names_.push_back("katana_motor1_pan_joint");
  joint_names_.push_back("katana_motor2_lift_joint");
  joint_names_.push_back("katana_motor3_lift_joint");
  joint_names_.push_back("katana_motor4_lift_joint");
  joint_names_.push_back("katana_motor5_wrist_roll_joint");

  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &FollowJointTrajectoryClient::jointStateCB, this);
  start_sub = nh_.subscribe("katana/start_trajectory", 1, &FollowJointTrajectoryClient::startTrajectoryCallback, this);
  start_part2_sub = nh_.subscribe("katana/start_trajectory2", 1, &FollowJointTrajectoryClient::startTrajectoryBackCallback, this);
  end_part2_pub = p.advertise<std_msgs::Bool>("katana/retracted2", 1);
  end_pub = p.advertise<std_msgs::Bool>("katana/retracted", 1);
  
  spinner_.start();
  
  // wait for action server to come up
  while (!traj_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the follow_joint_trajectory server");
  }
    ROS_INFO("Received trajectory client server.");  
  
  // wait for action server to come up
  while (!gripper_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the gripper_grasp_posture_controller server");
  }
    ROS_INFO("Received gripper_grasp server.");
}

FollowJointTrajectoryClient::~FollowJointTrajectoryClient()
{
}

//static katana_tutorials::FollowJointTrajectoryClient arm_static;
// Start trajectory when requested
void FollowJointTrajectoryClient::startTrajectoryCallback(const std_msgs::Bool::ConstPtr& msg)
{
      
      // Publisher end_pub publishes a signal to navigation when the katana ends trajectory
      std_msgs::Bool pub_msg;
      pub_msg.data = 1;
      // Init the TrajectoryClient
      //arm.send_gripper_action(GRASP);
        // Start the trajectory
      startTrajectory(makeArmUpTrajectory());
      while (!getState().isDone()) //&& ros::ok())
      {
	  // Suspend execution of the calling thread for (atleast) usec microseconds.
	  // This means, the trajectory will be completed even if the sigterm is sent to this node.
	  usleep(50000);
      }
      
      // publish a signal, that notifies the end of trajectory
     end_pub.publish(pub_msg);
}

// Start trajectory when requested
void FollowJointTrajectoryClient::startTrajectoryBackCallback(const std_msgs::Bool::ConstPtr& msg)
{

      // Publisher end_pub publishes a signal to navigation when the katana ends trajectory
      std_msgs::Bool pub_msg;
      pub_msg.data = 1;
      ROS_INFO("Calling back trajectory.");
      // Init the TrajectoryClient
      
      //arm.send_gripper_action(GRASP);
      // Start the trajectory
      startTrajectory(makeArmUpTrajectoryBack());
      while (!getState().isDone())
      {
	  // Suspend execution of the calling thread for (atleast) usec microseconds.
	  // This means, the trajectory will be completed even if the sigterm is sent to this node.
	  usleep(50000);
      }
      
      // publish a signal, that notifies the end of trajectory
      end_part2_pub.publish(pub_msg);
}

void FollowJointTrajectoryClient::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> ordered_js;

  ordered_js.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    bool found = false;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
      if (joint_names_[i] == msg->name[j])
      {
        ordered_js[i] = msg->position[j];
        found = true;
        break;
      }
    }
    if (!found)
      return;
  }

  ROS_INFO_ONCE("Got joint state!");
  current_joint_state_ = ordered_js;
  got_joint_state_ = true;
}

//! Sends the command to start a given trajectory
void FollowJointTrajectoryClient::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  traj_client_.sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectory()
{
  const size_t NUM_TRAJ_POINTS = 3;
  const size_t NUM_JOINTS = 5;

  // positions after calibration
  std::vector<double> calibration_positions(NUM_JOINTS);
  calibration_positions[0] = -2.90;
  calibration_positions[1] = 2.10;
  calibration_positions[2] = -2.15;
  calibration_positions[3] = -1.91;
  calibration_positions[4] = -2.87;

  // arm pointing forward
  std::vector<double> backwards_positions(NUM_JOINTS);
  backwards_positions[0] = 0.047;
  backwards_positions[1] = 1.0;
  backwards_positions[2] = -0.23;
  backwards_positions[3] = -0.92;
  backwards_positions[4] = -1.57;

//0.047342055960345686, 2.159243669302496, -0.2366207493507151, -0.9283515374436089, -1.5740006422184796, -0.054775763491041185, -0.054775763491041185

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(ind);
  trajectory.points[ind].positions = current_joint_state_;
 
  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(ind+1);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(ind+2);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = backwards_positions;
  
  
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;//filterJointTrajectory(trajectory);
  return goal;
  
}

control_msgs::FollowJointTrajectoryGoal FollowJointTrajectoryClient::makeArmUpTrajectoryBack()
{
  const size_t NUM_TRAJ_POINTS = 2;
  const size_t NUM_JOINTS = 5;

  // positions after calibration
  std::vector<double> calibration_positions(NUM_JOINTS);
  calibration_positions[0] = 0.5;
  calibration_positions[1] = 2.10;
  calibration_positions[2] = -2.15;
  calibration_positions[3] = -1.91;
  calibration_positions[4] = -2.87;

  // arm pointing forward
  std::vector<double> backwards_positions(NUM_JOINTS);
  backwards_positions[0] = 0.047;
  backwards_positions[1] = 2.15;
  backwards_positions[2] = -0.23;
  backwards_positions[3] = -0.92;
  backwards_positions[4] = -1.57;

//0.047342055960345686, 2.159243669302496, -0.2366207493507151, -0.9283515374436089, -1.5740006422184796, -0.054775763491041185, -0.054775763491041185

  trajectory_msgs::JointTrajectory trajectory;

  for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep())
  {
    ROS_DEBUG("waiting for joint state...");

    if (!ros::ok())
      exit(-1);
  }

  // First, the joint names, which apply to all waypoints
  trajectory.joint_names = joint_names_;

  trajectory.points.resize(NUM_TRAJ_POINTS);

  // trajectory point:
  int ind = 0;
  trajectory.points[ind].time_from_start = ros::Duration(ind);
  trajectory.points[ind].positions = current_joint_state_;

  // trajectory point:
  ind++;
  trajectory.points[ind].time_from_start = ros::Duration(ind+7);
  trajectory.points[ind].positions.resize(NUM_JOINTS);
  trajectory.points[ind].positions = calibration_positions;

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;//filterJointTrajectory(trajectory);
  return goal;
  
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState FollowJointTrajectoryClient::getState()
{
  return traj_client_.getState();
}

/* trajectory_msgs::JointTrajectory FollowJointTrajectoryClient::filterJointTrajectory(
    const trajectory_msgs::JointTrajectory &input)
{
  ros::service::waitForService("trajectory_filter/filter_trajectory");
  arm_navigation_msgs::FilterJointTrajectory::Request req;
  arm_navigation_msgs::FilterJointTrajectory::Response res;
  ros::ServiceClient filter_trajectory_client_ = nh_.serviceClient<arm_navigation_msgs::FilterJointTrajectory>(
      "trajectory_filter/filter_trajectory");

  req.trajectory = input;
  req.allowed_time = ros::Duration(1.0);

  if (filter_trajectory_client_.call(req, res))
  {
    if (res.error_code.val == res.error_code.SUCCESS)
      ROS_INFO("Requested trajectory was filtered");
    else
      ROS_WARN("Requested trajectory was not filtered. Error code: %d", res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to filter trajectory failed %s", filter_trajectory_client_.getService().c_str());
  }

  return res.trajectory;
} */

 bool FollowJointTrajectoryClient::send_gripper_action(int goal_type)
{
  GCG goal;

  switch (goal_type)
  {
    case GRASP:
      goal.command.position = -0.44; 
      // leave velocity and effort empty
      break;

    case RELEASE:
      goal.command.position = 0.3; 
      // leave velocity and effort empty
      break;

    default:
      ROS_ERROR("unknown goal code (%d)", goal_type);
      return false;
  }

  bool finished_within_time = false;
  gripper_.sendGoal(goal);
  finished_within_time = gripper_.waitForResult(ros::Duration(10.0));
  if (!finished_within_time)
  {
    gripper_.cancelGoal();
    ROS_WARN("Timed out achieving goal!");
    return false;
  }
  else
  {
    actionlib::SimpleClientGoalState state = gripper_.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
      ROS_INFO("Action finished: %s",state.toString().c_str());
    else
      ROS_WARN("Action failed: %s",state.toString().c_str());

    return success;
  }
 }

} /* namespace katana_tutorials */

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "follow_joint_trajectory_client");
  // Subscriber listens for signal from navigation to start trajectory
  ros::NodeHandle n;

  // Call constructor and create master object
  katana_tutorials::FollowJointTrajectoryClient armGripper;
  armGripper.send_gripper_action(GRASP);
  // start ros while loop
  ros::spin();
  return 0;
}
