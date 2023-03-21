import rospy
from std_msgs.msg import Float32MultiArray
import triad_openvr
from tf.transformations import quaternion_multiply
from scipy.spatial.transform import Rotation as R

from tf.transformations import euler_from_matrix

v = triad_openvr.triad_openvr()
import math
def float_list_publisher():
    # Initialize the ROS node
    rospy.init_node('float_list_publisher')

    # Create a publisher object that sends data on the "float_list" topic
    pub = rospy.Publisher('tracker_pose', Float32MultiArray, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(90)
    print("start to publish nZ euler")
    print("topic name : tracker_pose")
    print("node name : float_list_publisher")
    while not rospy.is_shutdown():
        ##### Create a Float32MultiArray message object
        msg = Float32MultiArray()

        ##### get device pose
        b = v.devices["tracker_1"].get_pose_quaternion()

        ##### to the local pose
        bong  = quaternion_multiply(b[3:], [-0.7071068, 0, 0, 0.7071068])
        
        ##### for euler publish
        o = R.from_quat(bong.tolist()).as_euler("YXZ", degrees = False).tolist()

        ##### ##### for nZ
        b[2] = -b[2]
        o[2] = -o[2]

        ##### publish
        # msg.data = b[:3] + bong.tolist() # for quat publish
        msg.data = b[:3] + o # for nZ euler publish

        # --------
        # r = R.from_quat(bong)

        # rr = euler_from_matrix(r.as_matrix())
        # print("1 : " , rr)
        # print(bong.tolist())
        # print("2 : " , r.as_euler("XYZ"))

        # r = r.as_matrix()
        # print(r.as_euler("xyz"))
        # print(bong)
        # if bong[1] < 0 : 
        #     theta_y = math.atan2(-math.sqrt(1-r[2,2]**2),r[2,2]) # neg
        #     theta_4 = math.atan2(-r[1,2],-r[0,2])
        #     theta_6 = math.atan2(-r[2,1], r[2,0])

        # elif bong[1] > 0 :
        #     theta_y = math.atan2(math.sqrt(1-r[2,2]**2),r[2,2]) # pos
        #     theta_4 = math.atan2(r[1,2],r[0,2])
        #     theta_6 = math.atan2(r[2,1], -r[2,0])

        # print(r.as_euler("XYZ"))
        # print([theta_4, theta_y,theta_6])
        # print(r.as_euler("y"))
        # Publish the message
        pub.publish(msg)

        # Sleep for the specified amount of time
        rate.sleep()

if __name__ == '__main__':
    try:
        float_list_publisher()
    except rospy.ROSInterruptException:
        pass