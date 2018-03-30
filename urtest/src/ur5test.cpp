#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "string.h"

#include "std_msgs/Float64MultiArray.h"

trajectory_msgs::JointTrajectory trajectory;
control_msgs::FollowJointTrajectoryGoal  goal;
trajectory_msgs::JointTrajectoryPoint way_point;

std_msgs::Float64MultiArray Q1[6] ;
std_msgs::Float64MultiArray Q2[6] ;
std_msgs::Float64MultiArray Q3[6] ;

float q[3][6]= {{2.2,0,-1.57,0.1,0,1},
                {2.2,0,-1.57,0.4,0,0},
                {1.5,0,-1.57,0,0,0}
               };

//std::string colour[4] = {"Blue", "Red", "Orange", "Yellow"};
std::string joint_names[6]= {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint","wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
float time_[3] = {2.0,3.0,6.0};
void move(void){



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5test");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_controller/follow_joint_trajectory", true);

  ROS_INFO("Starting ...");
  ROS_INFO("Waiting for action server to start.\n");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.\n");
  control_msgs::FollowJointTrajectoryGoal msg;
    for(int k=0;k<3;k++)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      for(int i=0;i<6;i++){
         point.positions.push_back(q[k][i]);
         point.velocities.push_back(0.0);
         point.time_from_start = ros::Duration(time_[k]);
      }

      msg.trajectory.points.push_back(point);
   }
    for(int j=0;j<6;j++){
      msg.trajectory.joint_names.push_back(joint_names[j].c_str());
    }
    msg.trajectory.header.stamp = ros::Time::now();

    ac.sendGoal(msg);
    while(!ac.waitForResult());


  ROS_INFO("Ending");
  return 0;
}
