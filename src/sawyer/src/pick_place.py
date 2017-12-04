#!/usr/bin/env python
import math
import struct
import sys
import copy
import Queue
import numpy as np
from numpy import linalg
from time import sleep

import rospy
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene

import intera_interface
from intera_interface import limb as robot_limb
from intera_interface import gripper as robot_gripper

def list_to_point(p):
    return Point(x=p[0], y=p[1], z=p[2])

def list_to_quaternion(q):
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class PickAndPlace(object):
    def __init__(self, limb, start_pose, hover_distance=0.15):
        self.start_pose = start_pose
        self.hover_distance = hover_distance
        self.limb = robot_limb.Limb(limb)
        self.gripper = robot_gripper.Gripper(limb)
        self.base = "base_marker"
        self.pick_obj = "obj_marker"
        self.dest = "dest_marker"
        self.destinations = []
        self.queue = Queue.Queue()
        self.tf = TransformListener()
        self.set_scene()
        self.set_limb_planner()
        self.enable_robot()
        self.set_destinations()
        print("Moving Sawyer Arm to Start Position")
        self.move_to_start()

    def set_limb_planner():
        self.right_arm = MoveGroupCommander("right_arm")
        self.right_arm.set_planner_id('RRTConnectkConfigDefault')
        self.right_arm.allow_replanning(True)
        self.right_arm.set_planning_time(7)

    def enable_robot():
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def set_scene():
        self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)
        self.add_collision_object('turtlebot',0.8,0.,-0.65,.5,1.5,.33)
        self.add_collision_object('right_wall',0.,0.65,0.,4.,.1,4.)
        self.add_collision_object('left_wall',0.,-0.8,0.,4.,.1,4.)
        self.add_collision_object('back_wall',-0.6,0.,0.,.1,4.,4.)
        self.add_collision_object('destination_table',.5,-.4,-.35,1,.5,.5)
        self.plan_scene = PlanningScene()
        self.plan_scene.is_diff = True
        self.scene_pub.publish(self.plan_scene)

    def add_collision_object(self,name,x,y,z,xs,ys,zs):
        self.scene.remove_world_object(name)
        o = PoseStamped()
        o.pose.position.x = x
        o.pose.position.y = y
        o.pose.position.z = z
        self.scene.attach_box('base', name, o, [xs,ys,zs])

    def set_destinations(self):
        while len(self.destinations) == 0:
            if self.tf.frameExists(self.base) and self.tf.frameExists(self.dest):
                point, quaternion = self.tf.lookupTransform(self.base, self.dest, self.tf.getLatestCommonTime(self.base, self.dest))
                position = list_to_point(point)
                self.destinations.append(Pose(position=position, orientation=self.start_pose.orientation))

    def add_new_objects_to_queue(self, sleep=True):
        print("Checking if " + self.base + " and " + self.pick_obj + " both exist.")
        if self.tf.frameExists(self.base) and self.tf.frameExists(self.pick_obj):
            if sleep:
                rospy.sleep(10.0)
                self.add_new_objects_to_queue(sleep=False)
            else:
                print(self.base + " and " + self.pick_obj + " both exist.")
                point, quaternion = self.tf.lookupTransform(self.base, self.pick_obj, self.tf.getLatestCommonTime(self.base, self.pick_obj))
                position = list_to_point(point)
                print("Adding " + self.pick_obj + " to queue")
                obj_location = Pose(position=position, orientation=self.start_pose.orientation)
                print("Picking Object from:", obj_location)
                self.queue.put(obj_location)

    def complete_pick_place(self):
        if not self.queue.empty():
            start_pose = self.queue.get()
            end_pose = self.destinations[0]
            print("\nPicking...")
            self.pick(start_pose)
            print("\nPlacing...")
            self.place(end_pose)
            print("\Resetting...")
            self.move_to_start()

    def guarded_move_to_joint_position(self, pose):
        self.right_arm.set_pose_target(pose)
        self.right_arm.go()

    def servo_to_pose(self, pose):
        # servo down to release
        self.guarded_move_to_joint_position(pose)

    def approach(self, pose):
        # approach with a pose the hover-distance above the requested pose
        approach = copy.deepcopy(pose)
        approach.position.z += self.hover_distance
        print("approaching:", approach)
        self.guarded_move_to_joint_position(approach)

    def retract(self):
        # retrieve current pose from endpoint
        current_pose = self.limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + self.hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        # servo up from current pose
        self.guarded_move_to_joint_position(ik_pose)

    def move_to_start(self):
        print("Moving the right arm to start pose...")
        self.guarded_move_to_joint_position(self.start_pose)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def gripper_open(self):
        self.gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self.gripper.close()
        rospy.sleep(1.0)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self.approach(pose)
        # servo to pose
        self.servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self.retract()

    def place(self, pose):
        # servo above pose
        self.approach(pose)
        # servo to pose
        self.servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self.retract()

def main():
    rospy.init_node('service_query')
    
    start_pose = Pose()
    start_pose.position.x = 0.561
    start_pose.position.y = 0.012
    start_pose.position.z = -0.152
    start_pose.orientation.x = 0.997
    start_pose.orientation.y = 0.070
    start_pose.orientation.z = 0.009
    start_pose.orientation.w = 0.004

    pnp = PickAndPlace(limb='right',start_pose=start_pose,hover_distance=0.12)

    while not rospy.is_shutdown():
        print("Looking for new objects to pick and place...")
        try:
            pnp.add_new_objects_to_queue()
            pnp.complete_pick_place()
        except Exception:
            print("Dest, Obj, or Base AR Tag not found. Retrying...")
    return 0
 
#Python's syntax for a main() method
if __name__ == '__main__':
    main()