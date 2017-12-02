#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from go_to_specific_point_on_map import GoToPose

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(data):

    #Print the contents of the message to the console
    print "Message:", data.x, data.y, "Received at:", rospy.get_time()



    navigator = GoToPose()
    navigator.goto(data.x, data.y)

    # if success:
    #     rospy.loginfo("Hooray, reached the desired pose")
    # else:
    #     rospy.loginfo("The base failed to reach the desired pose")

    # Sleep to give the last log messages time to be sent
    rospy.sleep(1)





#Define the method which contains the node's main functionality
def listener():
    print("hi")
    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('turtlebot_listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("turtlebot_dest", Point, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':

    listener()
