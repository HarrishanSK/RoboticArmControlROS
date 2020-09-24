#!/usr/bin/env python
#Student Name: Harrishan Sureshkumar
#Student ID: 150294245
import rospy
import random
from std_msgs.msg import String
from std_msgs.msg import Float32 

def square_size_generator():
    rospy.init_node('square_size_generator', anonymous=True)#initialise node
    print("Node1: Square Size Generator Activated") #Print message in terminal
    pub = rospy.Publisher('squareSizeGen', Float32, queue_size=10)#publisher for topic 'squareSizeGen'
    rate = rospy.Rate(0.05) # Must be 20secs use 0.05hz
    while not rospy.is_shutdown():#while rospy is not shutdown
	minVal = 0.05 #minimum value a random number can be
	maxVal = 0.20 #maximum value a random number can be

	square_size = random.uniform(minVal,maxVal) #random number between min and max values

	square_size = round(square_size, 6)
	
	rospy.loginfo(square_size) #uncomment to display values in terminal when running this node alone
	pub.publish(square_size) #publish on topic 'squareSizeGen'
        rate.sleep()#keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        square_size_generator()
    except rospy.ROSInterruptException:
        pass
