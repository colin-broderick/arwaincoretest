import sys
import rospy
import rosbag
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32


def from_position_log(filename):
    """
    Convert an ARWAIN position log file to a rosbag file for playback.
    """
    bag = rosbag.Bag(filename + "/replay.bag", "w")

    with open(filename + "/position.txt", "r") as f:
        for line in f.readlines()[1:]:
            try:
                t, x, y, z = line.split()
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.from_sec(float(t)/1e9)
                msg.header.frame_id = "world"
                msg.pose.position.x = float(x)
                msg.pose.position.y = float(y)
                msg.pose.position.z = float(z)
                msg.pose.orientation.w = 1
                msg.pose.orientation.x = 0
                msg.pose.orientation.y = 0
                msg.pose.orientation.z = 0
                bag.write("/arwain/node_2/position", msg, msg.header.stamp)
            except:
                pass

    with open(filename + "/stance.txt", "r") as f:
        for line in f.readlines()[1:]:
            try:
                t, freefall, entangled, attitude, stance = line.split()
                msg = Int32()

                msg.data = int(freefall)
                bag.write("/arwain/node_2/freefall", msg, rospy.Time.from_sec(float(t)/1e9))

                msg.data = int(attitude)
                bag.write("/arwain/node_2/attitude", msg, rospy.Time.from_sec(float(t)/1e9))
    
                msg.data = int(stance)
                bag.write("/arwain/node_2/stance", msg, rospy.Time.from_sec(float(t)/1e9))
            except:
                pass
    bag.close()


if __name__ == "__main__":
    from_position_log(sys.argv[1])
