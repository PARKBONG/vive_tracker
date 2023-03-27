import rospy
from std_msgs.msg import Float32MultiArray
import triad_openvr
from tf.transformations import quaternion_multiply
from scipy.spatial.transform import Rotation as R
import numpy as np
from tf.transformations import euler_from_matrix

v = triad_openvr.triad_openvr()
import math

class Interp():
    def __init__(self):
        self.old = [0,0,0]
        
    def euler(self,new):
        for i in range(3):
            dtheta = new[i] - self.old[i]
            rdtheta = R.from_euler('x', dtheta, degrees=False)
            interp_dtheta = rdtheta.as_euler('xyz', degrees=False)
            self.old[i] = self.old[i] + interp_dtheta[0]
        return self.old

def float_list_publisher():
    # Initialize the ROS node
    rospy.init_node('float_list_publisher')

    # Create a publisher object that sends data on the "float_list" topic
    pub = rospy.Publisher('tracker_pose', Float32MultiArray, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(90)
    print("start to publish without headset")
    print("topic name : tracker_pose")
    print("node name : float_list_publisher")

    Tracker_to_my = np.array([[-1,0,0],[0,0,-1],[0,-1,0]])
    my_to_sim = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    # tracker_to_my_trans = np.array([0,0.5, 1.5])
    tracker_to_my_trans = np.array([0,-0.6, -1.3])
    nYnZ = np.array([1,-1,-1])
    multiplier = 100
    more_than_2pi = Interp()
    while not rospy.is_shutdown():
        ##### Create a Float32MultiArray message object
        msg = Float32MultiArray()

        ##### get device pose
        b = v.devices["tracker_1"].get_pose_quaternion()
        
        # Real # -nY, -nZ
        rot_my = (nYnZ * R.from_matrix(Tracker_to_my @ R.from_quat(b[3:]).as_matrix()).as_euler("XYZ", degrees=False)).tolist()
        trans_my = (multiplier * nYnZ * ((Tracker_to_my @ np.array(b[:3]).T) + tracker_to_my_trans)).tolist()
        
        ##### expand domain
        # rot_my = more_than_2pi.euler(rot_my)

        ##### publish
        msg.data = trans_my + rot_my 

        pub.publish(msg)

        # Sleep for the specified amount of time
        rate.sleep()

if __name__ == '__main__':
    import sys
    try:
        float_list_publisher()
    except rospy.ROSInterruptException:
        pass

# orientation
#
# b = v.devices["tracker_1"].get_pose_quaternion()
# TOPVIEW
#
#       Z
#       ^
#       |
#  X <--*
#
#
# rot_my = R.from_matrix(Tracker_to_my @ R.from_quat(b[3:]).as_matrix()).as_euler("XYZ", degrees=False).tolist()
# TOPVIEW       
#
#       *--> X
#       |
#       v
#       Y