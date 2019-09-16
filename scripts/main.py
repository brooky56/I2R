import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import random

def talker():
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher', anonymous=True)
        rate = rospy.Rate(10) # 10hz
	hello_str = JointState()
	hello_str.header = Header()
        hello_str.name = ['base_link__link_01', 'link_01__link_02', 'link_02__link_03', 'link_03__link_04', 'link_04__link_05']
        hello_str.velocity = []
        hello_str.effort = []
	# Polnuy ROSkolbas for our bot 
       	while not rospy.is_shutdown():
		hello_str.header.stamp = rospy.Time.now()
		hello_str.position = [random.uniform(-3.14,3.14), random.uniform(0.0,0.5), random.uniform(0.0,0.75), random.uniform(0.0,0.75), random.uniform(-3.14,3.14)]
           	rospy.loginfo(hello_str)
           	pub.publish(hello_str)
           	rate.sleep()
  
if __name__ == '__main__':
	try:
		talker()
       	except rospy.ROSInterruptException:
          	pass
