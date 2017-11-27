#!/usr/bin/env python
import struct
import sys
import copy
import Queue

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import OrientationConstraint, Constraints
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from tf import TransformListener
import numpy as np
from numpy import linalg
from time import sleep

import intera_interface
from intera_interface import limb as robot_limb
from intera_interface import gripper as robot_gripper

def list_to_point(p):
    return Point(x=p[0], y=p[1], z=p[2])

def list_to_quaternion(q):
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class PickAndPlace(object):
    def __init__(self, limb, starting_joint_angles, hover_distance=0.15, base_id=4, dest_ids=[0, 1, 3, 5], verbose=True):
        self.starting_joint_angles = starting_joint_angles
        # TODO: The base_id and dest_ids can change when webcam_track is newly launched. Should be set by user input.
        self.base = base = "ar_marker_" + str(base_id)
        self.dest_ids = dest_ids
        self.destinations = []
        self.queue = Queue.Queue()
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = robot_limb.Limb(limb)
        self._gripper = robot_gripper.Gripper(limb)
        self.tf = TransformListener()
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        rospy.wait_for_service('compute_ik', 5.0)
        print("Getting robot state... ")
        # verify robot is enabled
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def set_destinations(self):
        for dest_id in self.dest_ids:
            dest = "ar_marker_" + str(dest_id)
            if self.tf.frameExists(self.base) and self.tf.frameExists(dest):
                point, quaternion = self.tf.lookupTransform(self.base, dest, self.tf.getLatestCommonTime(self.base, dest))
                position = list_to_point(point)
                orientation = list_to_quaternion(quaternion)
                self.destinations.append(Pose(position=position, orientation=orientation))

    def add_new_objects_to_queue(self):
        # TODO: ar_marker_6 hardcoded for now.
        pick_id = 5
        pick_obj = "ar_marker_" + str(pick_id)
        if self._verbose: print("Checking if " + self.base + " and " + pick_obj + " both exist.")
        if self.tf.frameExists(self.base) and self.tf.frameExists(pick_obj):
            if self._verbose: print(self.base + " and " + pick_obj + " both exist.")
            point, quaternion = self.tf.lookupTransform(self.base, pick_obj, self.tf.getLatestCommonTime(self.base, pick_obj))
            position = list_to_point(point)
            orientation = list_to_quaternion(quaternion)
            print("Adding " + pick_obj + " to queue")
            self.queue.put(Pose(position=position, orientation=orientation))

    def complete_pick_place(self):
        if not self.queue.empty():
            start_pose = self.queue.get()
            # TODO: This is hardcoded for now. Should be dynamically set according to something (color?)
            end_pose = self.destinations[0]
            print("\nPicking...")
            self.pick(start_pose)
            print("\nPlacing...")
            self.place(end_pose)

    def ik_request(self, pose):
        # Desired orientation for the end effector
        if self._verbose: print("Position:", pose.position)
        if self._verbose: print("Orientation:", pose.orientation)

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

    def move_to_start(self):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not self.starting_joint_angles:
            self.starting_joint_angles = dict(zip(self._joint_names, [0]*7))
        if self.starting_joint_angles:
            self._limb.move_to_joint_positions(self.starting_joint_angles)
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
    # TODO: May change so maybe need to set using user input
    starting_joint_angles = {'right_j0': 1.3476357421875,
                             'right_j1': 0.52880859375,
                             'right_j2': -0.567181640625,
                             'right_j3': -1.6188564453125,
                             'right_j4': -2.4296396484375,
                             'right_j5': -2.3929951171875,
                             'right_j6': -0.7705029296875}

    # An orientation for gripper fingers to be overhead and parallel to the obj
    # overhead_orientation = Quaternion(x=-0.075, y=0.996, z=0.035,w=-0.017)

    # block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    # block_poses.append(Pose(
    #     position=Point(x=0.765, y=0.160, z=-0.198),
    #     orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    # block_poses.append(Pose(
    #     position=Point(x=0.392, y=-0.694, z=-0.081),
    #     orientation=overhead_orientation))

    pnp = PickAndPlace(limb, starting_joint_angles, hover_distance)
    pnp.set_destinations()
    pnp.move_to_start()
    # idx = 0
    print(pnp.tf.getFrameStrings())
    while not rospy.is_shutdown():
        print("Looking for new objects to pick and place...")
        pnp.add_new_objects_to_queue()
        pnp.complete_pick_place()
    return 0
 
#Python's syntax for a main() method
if __name__ == '__main__':
    main()