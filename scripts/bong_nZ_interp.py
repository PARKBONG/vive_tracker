import rospy
from std_msgs.msg import Float32MultiArray
import triad_openvr
from tf.transformations import quaternion_multiply
from scipy.spatial.transform import Rotation as R

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

def float_list_publisher(orientation_order):
    # Initialize the ROS node
    rospy.init_node('float_list_publisher')

    # Create a publisher object that sends data on the "float_list" topic
    pub = rospy.Publisher('tracker_pose', Float32MultiArray, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(90)
    print("start to publish nZ euler with LARGER domain")
    print("topic name : tracker_pose")
    print("node name : float_list_publisher")

    more_than_2pi = Interp()
    while not rospy.is_shutdown():
        ##### Create a Float32MultiArray message object
        msg = Float32MultiArray()

        ##### get device pose
        b = v.devices["tracker_1"].get_pose_quaternion()

        ##### to the local pose
        bong  = quaternion_multiply(b[3:], [-0.7071068, 0, 0, 0.7071068])
        
        ##### for euler publish
        # o = R.from_quat(bong.tolist()).as_euler("YXZ", degrees = False).tolist()
        o = R.from_quat(bong.tolist()).as_euler(orientation_order, degrees = False).tolist()

        ##### ##### for nZ
        b[2] = -b[2]
        o[2] = -o[2]

        ##### expand domain
        o = more_than_2pi.euler(o)
        ##### publish
        # msg.data = b[:3] + bong.tolist() # for quat publish
        # print(o)
        msg.data = b[:3] + o # for nZ euler publish
        

        pub.publish(msg)

        # Sleep for the specified amount of time
        rate.sleep()

if __name__ == '__main__':
    import sys
    try:
        float_list_publisher(sys.argv[1])
    except rospy.ROSInterruptException:
        pass