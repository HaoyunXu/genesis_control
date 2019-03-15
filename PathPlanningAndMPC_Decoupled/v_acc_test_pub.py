import numpy as np
import rospy
from std_msgs.msg import Float32 as Float32Msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool as BoolMsg

def talker():
    pub = rospy.Publisher('/acc_state',Float32MultiArray , queue_size=2)
    rospy.init_node('v_acc_test_pub', anonymous=False)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(Float32MultiArray(data=[0.0,5.0,6.0]))
        rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
