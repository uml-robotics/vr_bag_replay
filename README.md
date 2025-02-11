# vr_bag_replay
This repository contains ROS scripts for recording and replaying ROS bag files in conjunction with the UML Robotics VR Interface project. The two key components in this project are the Rosbag Recorder and the Rosbag Replayer.

## Overview
This project is designed to facilitate recording and replaying ROS bag files that capture critical data from a robotic system used in the UML Robotics VR Interface. The Rosbag Recorder subscribes to several ROS topics and records the incoming data into a .bag file. The Rosbag Replayer allows the playback of recorded bags, reproducing the data streams at a set speed.

## Rosbag Recorder
The Rosbag Recorder is a C++ script that records data from various ROS topics and writes the messages to a bag file. It supports the following topics:

/joint_states
/functional_gripper_goal
/head_camera/depth_registered/points/filtered/throttled
/base_scan
/head_camera/rgb/image_raw/compressed
/gripper_goal/current
### How it Works
Service Server: The recorder has two services:
/start_bag: Starts recording to a new bag file.
/stop_bag: Stops recording and closes the current bag file.
Topics: It subscribes to a series of topics and writes received messages to the bag file.
File Naming: The script dynamically generates unique filenames for each new recording using a simple counter stored in the /bags/ directory.

## Rosbag Replayer
The Rosbag Replayer is a Python script that replays the recorded bag files. It subscribes to topics and republishes them at the original timestamps, optionally adjusting the speed of playback.

### How it Works
Replay Mode: It has several modes, such as stop, pause, and play, which control the flow of replayed data.
Replay Speed: The replay speed is configurable using a subscriber to set the speed dynamically.
Dynamic Topic Replication: The replayer republishes messages to new topics (prefixed with /replay).

## Usage
### Running the Recorder
Launch the recorder node:
'''bash
rosrun your_package bag_recorder.cpp

Start recording: To start recording, call the service:
'''bash
rosservice call /start_bag

Stop recording: To stop recording, call the service:
'''bash
rosservice call /stop_bag

### Running the Replayer
Launch the replayer node:

'''bash
rosrun your_package bag_replayer

Control replay: You can set the replay mode, speed, and replay file using the following services:
/set_current_replay: Set the active replay file.
/set_replay_mode: Set the replay mode (e.g., "play", "pause", "stop").
/set_replay_speed: Set the replay speed.
/set_current_time: Set the current replay time.

## Configuration
Record Topics: The recorder subscribes to a list of topics and records them. You can add or remove topics as needed.
Playback Topics: The replayer can be configured to replay a set of topics by modifying the repub_topics list in the replayer script.

## Dependencies
ROS: The project is designed for ROS1, tested with ROS Melodic.

