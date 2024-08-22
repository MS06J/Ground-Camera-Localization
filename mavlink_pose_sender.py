import rospy
from mavlink_sender import MavlinkSender
from pose_listener import PoseListener

sender = MavlinkSender()

sender.connect()
# set origin
sender.set_home()
sender.set_ekf_home()

listener = PoseListener(action=sender.send_pose)
rospy.spin()