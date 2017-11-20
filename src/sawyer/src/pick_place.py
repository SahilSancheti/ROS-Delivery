#!/usr/bin/env python
import struct
import sys
import copy

import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from time import sleep

import intera_interface
from intera_interface import limb as robot_limb
from intera_interface import gripper as robot_gripper

class PickAndPlace(object):
    def __init__(self, limb, request, hover_distance=0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = robot_limb.Limb(limb)
        self._gripper = robot_gripper.Gripper(limb)
        self.request = request
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
        print("Orientation:". pose.orientation)
        self.request.ik_request.pose_stamped.pose.position.x = pose.position.x
        self.request.ik_request.pose_stamped.pose.position.y = pose.position.y
        self.request.ik_request.pose_stamped.pose.position.z = pose.position.z
        self.request.ik_request.pose_stamped.pose.orientation.x = pose.orientation.x
        self.request.ik_request.pose_stamped.pose.orientation.y = pose.orientation.y
        self.request.ik_request.pose_stamped.pose.orientation.z = pose.orientation.z
        self.request.ik_request.pose_stamped.pose.orientation.w = pose.orientation.w
        try:
            #Send the request to the service
            response = self.compute_ik(self.request)
            if self._verbose:
                print("IK Reponse:")
                print(response)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        if self._verbose:
            print("Limb Joints:")
            print(limb_joints)
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

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
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
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
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    #Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right arm"
    request.ik_request.ik_link_name = "right_gripper"
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"

    limb = 'right'
    hover_distance = 0.15 # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_w0': 0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': -0.4999997247485215,
                             'right_e0': -1.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': -0.08000397926829805,
                             'right_s1': -0.9999781166910306}
    pnp = PickAndPlace(limb, request, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=-0.129),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
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