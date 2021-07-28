import rospy
from rospy.client import spin
from rospy.core import rospyinfo
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

def callback(msg):
    print(msg.data[5])

def setup():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("shimmer_imu_pub", Float32MultiArray, callback)    

    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Python node has been started!")
    setup()