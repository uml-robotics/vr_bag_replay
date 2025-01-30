#!/usr/bin/env python

import rospy
import rosbag
from datetime import datetime
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Time
import os

repub_topics = [
    #'/clock',
    '/head_camera/rgb/image_raw/compressed',
    '/head_camera/depth_registered/points/filtered/throttled',
    '/base_scan',
    '/gripper_goal/current',
    '/joint_states'
    ]
current_replay = ""

list_of_replays = []
current_time = 0.0
#current_duration = 1666048230 - 1666048219

#bag_start = 1666048219
#bag_end = 1666048230

replay_stop = "stop"
replay_play = "play"

replay_mode = "stop"

def set_current_mode(msg):
    global replay_mode
    global current_time
    rospy.loginfo("Setting current replay mode to")
    rospy.loginfo(msg.data)
    replay_mode = msg.data
    current_time = 0

def set_current_time(msg):
    global current_time
    rospy.loginfo("Setting current replay time to")
    rospy.loginfo(msg)
    current_time = msg.data#.secs
    
def set_current_replay(msg):
    rospy.loginfo("Setting Active replay ID to")
    rospy.loginfo(msg)
    current_replay = msg.data
    if current_replay in list_of_replays:
        load_bag("/bags/" + current_replay)
    else:
        rospy.loginfo("Invalid replay ID")
    
    
def load_bag(bag_file):
    global bag_start, bag_end, current_duration
    bag = rosbag.Bag(bag_file)
    bag_start = bag.get_start_time()
    bag_end = bag.get_end_time()
    current_duration = bag_end - bag_start
    
    for topic, msg, t in bag.read_messages( topics="/gripper_goal/current"):
        pub = rospy.Publisher("/replay" +topic, type(msg), queue_size=100)
        rospy.loginfo("We got a gripper goal!")
        break
    rospy.loginfo(current_duration)

def replay_bag(bag_file, _start_time, _end_time):
    global replay_mode, replay_stop, replay_play, current_time
    bag = rosbag.Bag(bag_file)
    #current_duration = bag.get_end_time() - bag.get_start_time()
    #rospy.loginfo(current_duration)
    _start_time = rospy.Time.from_sec(bag.get_start_time())
    _end_time = rospy.Time.from_sec(bag.get_end_time())
    #_start_time += current_time
    if(_end_time < _start_time):
        rospy.loginfo("End time is before start time")
        bag.close()
        return
    #start_time=_start_time, end_time=_end_time,
    for topic, msg, t in bag.read_messages( topics=repub_topics):
        if rospy.is_shutdown():
            rospy.loginfo("Shutdown signal received")
            bag.close()
            break
        if replay_mode == replay_stop:
            rospy.loginfo("Replay mode is stop")
            bag.close()
            break
        rospy.loginfo("Replaying message on topic ")
        rospy.loginfo(topic)
        rospy.loginfo(t.to_sec())
        pub = rospy.Publisher("/replay" +topic, type(msg), queue_size=100)
        pub.publish(msg)
        rospy.sleep(0.001)  # Adjust sleep time as needed
    replay_mode = replay_stop
    bag.close()
    
#def close_bag():
#    rospy.loginfo("Closing bag")
#    bag.close()

def list_files(directory):
    return os.listdir(directory)
    #files = []
    #for f in os.listdir(directory):
    #    if os.path.isfile(f):
    #        files.append(f)
    #return files

if __name__ == "__main__":
    #global list_of_replays
    rospy.init_node('bag_replay', anonymous=True)
    rospy.Subscriber('/set_current_replay', String, set_current_replay)
    rospy.Subscriber('/set_current_time', Int32, set_current_time)
    rospy.Subscriber('/set_replay_mode', String, set_current_mode)
    
    #rospy.Service()
    
    list_of_replays_pub = rospy.Publisher('/list_of_replays', String, queue_size=10)
    time_pub = rospy.Publisher('/replay_time', Time, queue_size=10)
    duration_pub = rospy.Publisher('/replay_duration', Time, queue_size=10)
    
    
    rate = rospy.Rate(10)
    list_of_replays_msg = ""#todo sccan bag directory to dynamically load
    list_of_replays = list_files("/bags")
    #print(list_of_replays)
    
    for i in range(0, len(list_of_replays)):
        list_of_replays_msg+=list_of_replays[i] + ","
    #list_of_replays_pub.publish(list_of_replays_msg)
    load_bag("/bags/" + list_of_replays[0]) #initialize with first bag to start
    while not rospy.is_shutdown():
        if(list_files("/bags") != list_of_replays):
            print("New bag detected, reloading")
            list_of_replays = list_files("/bags")
            list_of_replays_msg = ""
            for i in range(0, len(list_of_replays)):
                list_of_replays_msg+=list_of_replays[i] + ","
            
        current_time_msg = Time()
        current_time_msg.data.secs = current_time
        time_pub.publish(current_time_msg)
        
        current_duration_msg = Time()
        current_duration_msg.data.secs = current_duration
        duration_pub.publish(current_duration_msg)
        list_of_replays_pub.publish(list_of_replays_msg)

        if replay_mode == replay_play and current_time < current_duration:
            rospy.loginfo("Replaying bag")
            rospy.loginfo(list_of_replays)
            replay_bag("/bags/" + list_of_replays[current_replay], bag_start , bag_start)
            current_time += 1
        else:
            #rospy.loginfo("Not replaying bag")
            replay_mode = replay_stop
        rate.sleep()
#``` close_bag()
