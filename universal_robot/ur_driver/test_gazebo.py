#!/usr/bin/env python
import time
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

# Define 6 joint names here, you could find it in rostopic 
JOINT_NAMES = [xxx, xxx, xxx, xxx, xxx, xxx]

# Goal joint example , wrist should be zero
Q1 = [x,x,x,x,x,x]
Q2 = [x,x,x,x,x,x]
Q3 = [x,x,x,x,x,x]

client = None

def move():
    # Define the joint name in JointTrajectory message type
    # FollowJointTrajectoryGoal -> JointTrajectory -> joint_names
    g = XXX
    g.trajectory = XXX
    g.trajectory.joint_names = XXX
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(6.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    
def move_repeated():
    # Define the joint name in JointTrajectory message type
    # FollowJointTrajectoryGoal -> JointTrajectory -> joint_names
    g = XXX
    g.trajectory = XXX
    g.trajectory.joint_names = XXX
    
    d = 2.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for ur5_arm server..."
        client.wait_for_server()
        print "Connected to ur5_arm server"
        move()
        #move_repeated()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
