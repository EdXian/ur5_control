#include <vector>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_trajectory");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  group.setPoseReferenceFrame("/base");
  group.setPlanningTime(1);

  // Create a trajectory (vector of Eigen poses)
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
  Eigen::Affine3d pose (Eigen::Affine3d::Identity());

  // Square belonging to XY plane
  pose.linear() <<
      1, 0, 0,
      0,-1, 0,
      0, 0, -1;
  pose.translation() << 0.8, 0.1, -0.1;
  way_points_vector.push_back(pose);
  pose.translation() << 1.0, 0.1, -0.1;
  way_points_vector.push_back(pose);
  pose.translation() << 1.0, 0.3, -0.1;
  way_points_vector.push_back(pose);
  pose.translation() << 0.8, 0.3, -0.1;
  way_points_vector.push_back(pose);
  way_points_vector.push_back(way_points_vector[0]);

  // Copy the vector of Eigen poses into a vector of ROS poses
  std::vector<geometry_msgs::Pose> way_points_msg;
  way_points_msg.resize(way_points_vector.size());
  for (size_t i = 0; i < way_points_msg.size(); i++)
    tf::poseEigenToMsg(way_points_vector[i], way_points_msg[i]);

  moveit_msgs::RobotTrajectory robot_trajectory;
  group.computeCartesianPath(way_points_msg, 0.05, 15, robot_trajectory);

  // Execute trajectory
  ros::ServiceClient executeKnownTrajectoryServiceClient = node.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
      "/execute_kinematic_path");
  moveit_msgs::ExecuteKnownTrajectory srv;

  srv.request.wait_for_execution = true;
  srv.request.trajectory = robot_trajectory;
  executeKnownTrajectoryServiceClient.call(srv);

  ros::waitForShutdown();
  spinner.stop();
   return 0;
}
