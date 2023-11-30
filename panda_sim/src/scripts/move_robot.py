#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose

def main():
    # Initialize ROS node
    rospy.init_node('moveit_commander', anonymous=True)

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object (used to add and remove objects from the planning scene)
    scene = PlanningSceneInterface()

    # Instantiate a MoveGroupCommander for the robot's arm
    group_name = "panda_arm"  # Replace with your robot's MoveIt group name
    group = moveit_commander.MoveGroupCommander(group_name)

    # Set the planning time
    group.set_planning_time(10)  # Adjust this as needed

    # Define the target pose (you need to specify the pose of your end-effector)
    target_pose = Pose()
    target_pose.position.x = 0.43 # Replace with your desired x-coordinate
    target_pose.position.y = -0.438  # Replace with your desired y-coordinate
    target_pose.position.z = 0.4  # Replace with your desired z-coordinate
    target_pose.orientation.x = -0.932304811466142  # Replace with your desired orientation quaternion
    target_pose.orientation.y = 0.3362381127976848
    target_pose.orientation.z = -0.11189248387675493
    target_pose.orientation.w = 0.07233078232138017
    current_pose = group.get_current_pose().pose
    print(current_pose)
    target_pose.orientation = current_pose.orientation

    # # Set the target pose for the end-effector
    group.set_pose_target(target_pose)

    # Plan and execute the trajectory to the target pose
    plan = group.plan()
    group.go(wait=True)
    
    group.stop()
    # group.execute(plan)
    
    
    # Get the current joint angles
    # current_joint_values = group.get_current_joint_values()
    # print(current_joint_values)

    # # Modify the joint state (for example, add 0.1 radians to the first joint)
    # current_joint_values[6] = 1.9628646101943104  # Modify this value as needed for your use case

    # # # Set the modified joint state
    # group.set_joint_value_target(current_joint_values)
    # plan = group.plan()
    # group.go(wait=True)
    
    

    # Shutdown MoveIt Commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
