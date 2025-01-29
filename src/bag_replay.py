#!/bin/env python

import rospy
import rosbag
from datetime import datetime
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Time

current_replay = 0

list_of_replays = ["bags/replay.bag"]
current_time = 0
current_duration = 1666048230 - 1666048219

bag_start = 1666048219
bag_end = 1666048230

replay_stop = "stop"
replay_play = "play"
replay_mode = "stop"

def set_current_mode(msg):
    global replay_mode
    rospy.loginfo("Setting current replay mode to")
    rospy.loginfo(msg.data)
    replay_mode = msg.data

def set_current_time(msg):
    global current_time
    rospy.loginfo("Setting current replay time to")
    rospy.loginfo(msg)
    current_time = msg.data#.secs
    
def set_current_replay(msg):
    rospy.loginfo("Setting Active replay ID to")
    rospy.loginfo(msg)
    current_replay = msg.data
    
    

def replay_bag(bag_file, _start_time, _end_time):
    bag = rosbag.Bag(bag_file)
    _start_time = rospy.Time.from_sec(_start_time)
    _end_time = rospy.Time.from_sec(_end_time)
    
    for topic, msg, t in bag.read_messages(start_time=_start_time, end_time=_end_time, topics=['/clock','/head_camera/rgb/image_raw/compressed']):
        if rospy.is_shutdown():
            break
        rospy.loginfo("Replaying message on topic ")
        rospy.loginfo(topic)
        rospy.loginfo(t.to_sec())
        pub = rospy.Publisher("/replay" +topic, type(msg), queue_size=10)
        pub.publish(msg)
        #rospy.sleep(0.1)  # Adjust sleep time as needed

    bag.close()

if __name__ == "__main__":
    #global list_of_replays
    rospy.init_node('bag_replay', anonymous=True)
    rospy.Subscriber('/set_current_replay', Int32, set_current_replay)
    rospy.Subscriber('/set_current_time', Int32, set_current_time)
    rospy.Subscriber('/set_replay_mode', String, set_current_mode)
    list_of_replays_pub = rospy.Publisher('/list_of_replays', String, queue_size=10)
    time_pub = rospy.Publisher('/replay_time', Time, queue_size=10)
    duration_pub = rospy.Publisher('/replay_duration', Time, queue_size=10)
    
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        list_of_replays_msg = ""
        for i in range(0, len(list_of_replays)):
            list_of_replays_msg+=list_of_replays[i] + ","
        list_of_replays_pub.publish(list_of_replays_msg)
        
        current_time_msg = Time()
        current_time_msg.data.secs = current_time
        time_pub.publish(current_time_msg)
        
        current_duration_msg = Time()
        current_duration_msg.data.secs = current_duration
        duration_pub.publish(current_duration_msg)
        print(replay_mode)
        #print(replay_play)
        if replay_mode == replay_play and current_time < current_duration:
            rospy.loginfo("Replaying bag")
            rospy.loginfo(list_of_replays)
            replay_bag(list_of_replays[current_replay], bag_start + current_time, bag_start + current_time + 1)
            current_time += 1
        else:
            rospy.loginfo("Not replaying bag")
            replay_mode = replay_stop
        
        rate.sleep()
    
    #bag_file = 'bags/replay.bag'
    #start_time = datetime.strptime('2022-10-17 19:10:23', '%Y-%m-%d %H:%M:%S').timestamp()
    #end_time = datetime.strptime('2022-10-17 19:10:23', '%Y-%m-%d %H:%M:%S').timestamp()
    #start_time = 1666048219;
    #end_time = 1666048230
    #replay_bag(bag_file, start_time, end_time)