# This function generates the velocity profile for MPC. The velocity profile is an
# array of velocity values, which illustrates the change of velocity from current
# one to that of front car. The size of array is the horizon of MPC.

# The inputs of this function are vf (the velocity of front car), ve (the velocity 
# of ego car), and d (the distance between two cars). It will return velocity 
# profile only in safe conditions (distance is greater than safe distance, and 
# deacceleration is in the max deacceleration boundary). Otherwise it will publish
# a True boolean message as a ROS node called "takeover_msg" to a ROS topic called 
# "takeover_bool".

#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Bool

def v_ref_gen(vf,ve,d):
    miu = 0.8; # The coefficient of friction
    g = 9.8; # Gravity constant
    N = 8; # MPC horizon
    v_ref = ve*np.ones(N); # Initialize the velocity profile as all ego velocity
    
    d_brake = (ve-vf)**2/(2*miu*g); # The braking distance formula, assuming 
                                    # change of kinetic energy fullyconverted to 
                                    # the work done by friction, skid only
    
    d_safe = d_brake + abs(ve-vf)*1; # The safe distance is longer than assumed 
                                     # braking distance, adding one second to add
                                     # more safety consideration
    if (vf < ve):
        v_ref[0] = ve;
        v_ref[N-1] = vf;
        
        if d >= d_safe:
            deaccl = (vf**2-ve**2)/(2*(d-d_safe)); # Find the deacceleration value
                                                   # to make car deaccelerate to be
                                                   # the front car velocity while 
                                                   # keep safety distance
            
            if deaccl >= -2: # Check the bound for deacceleration
                for i in range(1,N-1):
                    v_ref[i] = v_ref[i-1]-(ve-vf)/(N-2); # Do linear deacceleration
                                                         # to be the front car velocity
            
            else: # Send message to driver take-over
                pub = rospy.Publisher('takeover_bool', Bool, queue_size=10) 
                rospy.init_node('takeover_msg', anonymous=True) 
                rate = rospy.Rate(10) # 10hz
                
                while not rospy.is_shutdown():
                    driver_bool = True
                    rospy.loginfo(driver_bool)
                    pub.publish(driver_bool)
                    rate.sleep()
                    
                if __name__ == '__main__':
                    try:
                        talker()
                    except rospy.ROSInterruptException:
                        pass
                    
        if d < d_safe: # Send message to driver take-over
            pub = rospy.Publisher('takeover_bool', Bool, queue_size=10) 
            rospy.init_node('takeover_msg', anonymous=True)  
            rate = rospy.Rate(10) # 10hz
            
            while not rospy.is_shutdown():
                driver_bool = True
                rospy.loginfo(driver_bool)
                pub.publish(driver_bool)
                rate.sleep()
            
            if __name__ == '__main__':
                try:
                    talker()
                except rospy.ROSInterruptException:
                    pass
                
    else:
        pass
    
    return v_ref

