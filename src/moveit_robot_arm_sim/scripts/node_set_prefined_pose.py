#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

# Include the necessary libraries 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from math import pi, sqrt
from std_msgs.msg import String

class MyRobot:

    def __init__(self, group_name):
        # Initialize the moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_predefined_pose', anonymous=True)

        # Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot
        self._scene = moveit_commander.PlanningSceneInterface()

        # Define the move group for the robot
        self._planning_group = group_name
        # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        # Create action client for the "Execute Trajectory" action server
        self._execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        # Get the planning frame, end effector link, and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Print the info
        rospy.loginfo('\033[95m' + "Planning Frame: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        if arg_pose_name not in self._group.get_named_targets():
            rospy.logerr("Named target '{}' does not exist".format(arg_pose_name))
            return

        # Set the predefined position as the named joint configuration as the goal to plan for the move group
        self._group.set_named_target(arg_pose_name)

        # Plan to the desired joint-space goal using the default planner
        plan_success, plan, planning_time, error_code = self._group.plan()

        if not plan_success:
            rospy.logerr("Planning failed for pose: {}".format(arg_pose_name))
            return

        # Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # Update the trajectory in the goal message
        goal.trajectory = plan
        # Send the goal to the action server
        self._execute_trajectory_client.send_goal(goal)
        self._execute_trajectory_client.wait_for_result()

        # Print the current pose
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def __del__(self):
        # When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():
    # Create a new arm object from the MyRobot class
    arm = MyRobot("Arm_group")
    hand = MyRobot("hand_group")

    # Check available named targets for debugging purposes
    rospy.loginfo("Available named targets for arm: {}".format(arm._group.get_named_targets()))
    rospy.loginfo("Available named targets for hand: {}".format(hand._group.get_named_targets()))

    # Call the function to set the position to "zero_pose"
    if "zero_pose" in arm._group.get_named_targets():
        arm.set_pose("zero_pose")
    else:
        rospy.logerr("Named target 'zero_pose' does not exist in the arm group.")
    rospy.sleep(2)

    if "straight_up" in arm._group.get_named_targets():
        arm.set_pose("straight_up")
    else:
        rospy.logerr("Named target 'straight_up' does not exist in the arm group.")
    rospy.sleep(2)

    if "hand_opened" in hand._group.get_named_targets():
        hand.set_pose("hand_opened")
    else:
        rospy.logerr("Named target 'hand_opened' does not exist in the hand group.")
    rospy.sleep(2)

    while not rospy.is_shutdown():
        if "lift_object" in arm._group.get_named_targets():
            arm.set_pose("lift_object")
        else:
            rospy.logerr("Named target 'lift_object' does not exist in the arm group.")
        rospy.sleep(2)

        if "pick_object" in arm._group.get_named_targets():
            arm.set_pose("pick_object")
        else:
            rospy.logerr("Named target 'pick_object' does not exist in the arm group.")
        rospy.sleep(2)

        if "hand_closed" in hand._group.get_named_targets():
            hand.set_pose("hand_closed")
        else:
            rospy.logerr("Named target 'hand_closed' does not exist in the hand group.")
        rospy.sleep(2)

        if "drop_object" in arm._group.get_named_targets():
            arm.set_pose("drop_object")
        else:
            rospy.logerr("Named target 'drop_object' does not exist in the arm group.")
        rospy.sleep(2)

        if "hand_opened" in hand._group.get_named_targets():
            hand.set_pose("hand_opened")
        else:
            rospy.logerr("Named target 'hand_opened' does not exist in the hand group.")
        rospy.sleep(2)

    # Delete the arm and hand objects at the end of the code
    del arm
    del hand


if __name__ == '__main__':
    main()

