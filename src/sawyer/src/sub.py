#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage
import numpy as np
#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(data):

    #Print the contents of the message to the console
    if data.transforms[0].child_frame_id.startswith("ar_marker_"):
        print("AR Tag",data.transforms[0].child_frame_id)
        print("Translation",data.transforms[0].transform.translation)
        print("Orientation",data.transforms[0].transform.rotation)
    # print "Message:", data.x, data.y, "Received at:", rospy.get_time()
        x = data.transforms[0].transform.rotation.x
        y = data.transforms[0].transform.rotation.y
        z = data.transforms[0].transform.rotation.z
        w = data.transforms[0].transform.rotation.w
        alpha = np.arccos(w)*2
        betax = np.arccos(x/np.sin(alpha/2))
        betay = np.arccos(y/np.sin(alpha/2))
        betaz = np.arccos(z/np.sin(alpha/2))
        print(betax,betay,betaz)
    # if success:
    #     rospy.loginfo("Hooray, reached the desired pose")
    # else:
    #     rospy.loginfo("The base failed to reach the desired pose")

    # Sleep to give the last log messages time to be sent
    rospy.sleep(1)





#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('artag_position_listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("tf", TFMessage, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':

    listener()
