# Name: Sivakar Sivarajah
#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from AR_week8_test.msg import length_params

#Node 1 Publisher

def square_size_generator():
    pub = rospy.Publisher('length', length_params, queue_size=10)
    rospy.init_node('lenght_generator', anonymous=True)# initilaize the node
    rate = rospy.Rate(0.05) # 0.05hz it will publish every 20 seconds

    while not rospy.is_shutdown():

     	length = length_params()
    	length = random.uniform(0.05,0.20) #creates a random length from 0.05 to 0.20 
	rospy.loginfo(length)
    	pub.publish(length) # publish the length
        rate.sleep()

if __name__ == '__main__':
    try:
        square_size_generator()
    except rospy.ROSInterruptException:
        pass

