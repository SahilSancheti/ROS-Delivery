#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
# from my_chatter.msg import TimestampString
#Import the String message type from the /msg directory of
#the std_msgs package.
# from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import String
#Define the method which contains the main functionality of the node.
def talker():

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('turtle_talker', anonymous=True)

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type std_msgs/String to the topic /chatter_talk
  pub = rospy.Publisher('turtlebot_dest', Point, queue_size=10)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    # Construct a string that we want to publish
    # (In Python, the "%" operator functions similarly
    #  to sprintf in C or MATLAB)

    #-1.44 3.43
    pub_string = raw_input("Please enter a coordinate x y (Enter \'s\':Sawyer Location, \'i\':Init Location) <Enter>:\n").split()
    print(pub_string,type(pub_string), type(pub_string[0]))
    if len(pub_string)==1:
      if (pub_string[0].strip().lower() =="s"):
        pub_string = [-1,3.43]
      # print(pub_string,type(pub_string), type(pub_string[0]))
      elif (pub_string[0].strip().lower() =="i"):
        pub_string  = [-.6, .778]
    # Publish our string to the 'chatter_talk' topic
    pub.publish(Point(float(pub_string[0]), float(pub_string[1]), 0))
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  try:
    talker()
  except rospy.ROSInterruptException: pass
