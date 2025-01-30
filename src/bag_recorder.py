#!/usr/bin/env python

import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Empty, EmptyResponse

#Topics To Record
#JointStates
#SensorMsgsImage

class RosbagRecorder:
    def __init__(self):
        rospy.init_node('rosbag_recorder', anonymous=True)
        start_service = rospy.Service('/start_bag', Empty, self.start_record)
        stop_service = rospy.Service('/stop_bag', Empty, self.stop_record)
        rospy.on_shutdown(self.shutdown_hook)

    def jointStateCallback(self, msg):
        if self.write_to_bag:
            self.bag.write('/joint_states', msg)
        
    def pointCloudCallback(self, msg):
        if self.write_to_bag:
            self.bag.write('/head_camera/depth_registered/points/filtered/throttled/', msg)
        
    def laserCallback(self, msg):
        if self.write_to_bag:
            self.bag.write('/base_scan', msg)
        
    def imageCallback(self, msg):
        if self.write_to_bag:
            self.bag.write('/head_camera/rgb/image_raw/compressed', msg)
    
    def goalCallback(self, msg):
        if self.write_to_bag:
            self.bag.write('/gripper_goal/current', msg)
    
    def shutdown_hook(self):
        rospy.loginfo("Shutting down and closing bag file.")
        #self.bag.close()
        
    def start_record(self, req):
        rospy.loginfo("Recording to output.bag")
        self.write_to_bag = True
        self.bag = rosbag.Bag('/bags/output.bag', 'w')
        self.joint_state_sub =  rospy.Subscriber('/joint_states', JointState, self.jointStateCallback)
        self.point_cloud_sub =  rospy.Subscriber('/head_camera/depth_registered/points/filtered/throttled', PointCloud2, self.pointCloudCallback)
        self.laser_scan_sub =  rospy.Subscriber('/base_scan', LaserScan, self.laserCallback)
        #self.image_sub =  rospy.Subscriber('/head_camera/rgb/image_raw/compressed', CompressedImage, self.imageCallback)
        self.gripper_goal_sub =  rospy.Subscriber('/gripper_goal/current', PoseArray, self.goalCallback)
        return EmptyResponse()

    def stop_record(self, req):
        rospy.loginfo("Stopping recording to output.bag")
        self.write_to_bag = False
        self.joint_state_sub.unregister()
        self.point_cloud_sub.unregister()
        self.laser_scan_sub.unregister()
        #self.image_sub.unregister()
        self.gripper_goal_sub.unregister()
        self.bag.close()
        return EmptyResponse()


if __name__ == '__main__':
    recorder = RosbagRecorder()
    rospy.spin()
