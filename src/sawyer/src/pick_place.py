#!/usr/bin/env python
import struct
import sys
import copy

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import OrientationConstraint, Constraints
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import numpy as np
from numpy import linalg
from time import sleep

import intera_interface
from intera_interface import limb as robot_limb
from intera_interface import gripper as robot_gripper

class PickAndPlace(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = robot_limb.Limb(limb)
        self._gripper = robot_gripper.Gripper(limb)
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        rospy.wait_for_service('compute_ik', 5.0)
        print("Getting robot state... ")
        # verify robot is enabled
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def ik_request(self, pose):
        # Desired orientation for the end effector
        print("Position:", pose.position)
        print("Orientation:", pose.orientation)

        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        request.ik_request.pose_stamped.pose.position.x = pose.position.x
        request.ik_request.pose_stamped.pose.position.y = pose.position.y
        request.ik_request.pose_stamped.pose.position.z = pose.position.z
        request.ik_request.pose_stamped.pose.orientation.x = pose.orientation.x
        request.ik_request.pose_stamped.pose.orientation.y = pose.orientation.y
        request.ik_request.pose_stamped.pose.orientation.z = pose.orientation.z
        request.ik_request.pose_stamped.pose.orientation.w = pose.orientation.w

        right_arm = MoveGroupCommander(request.ik_request.group_name)
        right_arm.set_planner_id('RRTConnectkConfigDefault')
        right_arm.set_planning_time(10)

        # Setting position and orientation target
        right_arm.set_pose_target(request.ik_request.pose_stamped)

        orien_const = OrientationConstraint()
        orien_const.link_name = "right_gripper";
        orien_const.header.frame_id = "base";
        orien_const.orientation.x = pose.orientation.x;
        orien_const.orientation.y = pose.orientation.y;
        orien_const.orientation.z = pose.orientation.z;
        orien_const.orientation.w = pose.orientation.w;
        orien_const.absolute_x_axis_tolerance = 0.1;
        orien_const.absolute_y_axis_tolerance = 0.1;
        orien_const.absolute_z_axis_tolerance = 0.1;
        orien_const.weight = 1.0;
        consts = Constraints()
        consts.orientation_constraints = [orien_const]
        right_arm.set_path_constraints(consts)

        right_plan = right_arm.plan()

        return right_arm, right_plan
        # print("Group:")
        # print(right_arm)
        # # Plan IK and execute
        # # group.go()
        # try:
        #     #Send the request to the service
        #     response = self.compute_ik(request)
        #     if self._verbose:
        #         print("IK Reponse:")
        #         print(response)
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e
        #     return False
        # # Format solution into Limb API-compatible dictionary
        # limb_joints = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        # if self._verbose:
        #     print("Limb Joints:")
        #     print(limb_joints)
        # return limb_joints

    def _guarded_move_to_joint_position(self, limb, plan):
        if plan:
            limb.execute(plan)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def _servo_to_pose(self, pose):
        # servo down to release
        limb, plan = self.ik_request(pose)
        self._guarded_move_to_joint_position(limb, plan)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        limb, plan = self.ik_request(approach)
        self._guarded_move_to_joint_position(limb, plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        limb, plan = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(limb, plan)

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        if start_angles:
            self._limb.move_to_joint_positions(start_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def main():
    print("Moving Sawyer Arm to Start Position")
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    limb = 'right'
    hover_distance = 0.15 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.7147353515625,
                             'right_j1': 1.2915859375,
                             'right_j2': -1.6490263671875,
                             'right_j3': -1.994041015625,
                             'right_j4': -2.76866796875,
                             'right_j5': -1.9145673828125,
                             'right_j6': -0.463185546875}

    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.075,
                             y=0.996,
                             z=0.035,
                             w=-0.017)

    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.765, y=0.160, z=-0.198),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.392, y=-0.694, z=-0.081),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        print("\nPicking...")
        pnp.pick(block_poses[idx])
        print("\nPlacing...")
        idx = (idx+1) % len(block_poses)
        pnp.place(block_poses[idx])
    return 0
 
#Python's syntax for a main() method
if __name__ == '__main__':
    main()