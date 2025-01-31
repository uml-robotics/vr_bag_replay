#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>

class RosbagRecorder {
public:
    RosbagRecorder() : write_to_bag(false) {
        ros::NodeHandle nh;
        start_service = nh.advertiseService("/start_bag", &RosbagRecorder::startRecord, this);
        stop_service = nh.advertiseService("/stop_bag", &RosbagRecorder::stopRecord, this);
    }

    void spin()
    {
        ros::spin();
    }

    bool startRecord(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        std::string bag_file = getNextBag();
        ROS_INFO("Recording to %s", bag_file.c_str());
        write_to_bag = true;
        bag.open(bag_file, rosbag::bagmode::Write);
        joint_state_sub = nh.subscribe("/joint_states", 1000, &RosbagRecorder::jointStateCallback, this);
        gripper_state_sub = nh.subscribe("/functional_gripper_goal", 1000, &RosbagRecorder::gripperStateCallback, this);
        point_cloud_sub = nh.subscribe("/head_camera/depth_registered/points/filtered/throttled/", 1000, &RosbagRecorder::pointCloudCallback, this);
        laser_sub = nh.subscribe("/base_scan", 1000, &RosbagRecorder::laserCallback, this);
        image_sub = nh.subscribe("/head_camera/rgb/image_raw/compressed", 1000, &RosbagRecorder::imageCallback, this);
        goal_sub = nh.subscribe("/gripper_goal/current", 1000, &RosbagRecorder::goalCallback, this);
        return true;
    }

    bool stopRecord(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ROS_INFO("Stopping recording.");
        write_to_bag = false;
        bag.close();
        joint_state_sub.shutdown();
        gripper_state_sub.shutdown();
        point_cloud_sub.shutdown();
        laser_sub.shutdown();
        image_sub.shutdown();
        goal_sub.shutdown();
        return true;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (write_to_bag) {
            bag.write("/joint_states", ros::Time::now(), msg);
        }
    }
    
    void gripperStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        if (write_to_bag) {
            bag.write("/functional_gripper_goal", ros::Time::now(), msg);
        }
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (write_to_bag) {
            bag.write("/head_camera/depth_registered/points/filtered/throttled/", ros::Time::now(), msg);
        }
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (write_to_bag) {
            bag.write("/base_scan", ros::Time::now(), msg);
        }
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
        if (write_to_bag) {
            bag.write("/head_camera/rgb/image_raw/compressed", ros::Time::now(), msg);
        }
    }

    void goalCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (write_to_bag) {
            bag.write("/gripper_goal/current", ros::Time::now(), msg);
        }
    }

    std::string getNextBag() {
        boost::filesystem::path dir("/bags/");
        if (!boost::filesystem::exists(dir)) {
            boost::filesystem::create_directory(dir);
        }
        int count = std::distance(boost::filesystem::directory_iterator(dir), boost::filesystem::directory_iterator());
        return "/bags/output" + std::to_string(count + 1) + ".bag";
    }

private:
    ros::NodeHandle nh;
    ros::ServiceServer start_service;
    ros::ServiceServer stop_service;
    ros::Subscriber joint_state_sub;
    ros::Subscriber gripper_state_sub;
    ros::Subscriber point_cloud_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber image_sub;
    ros::Subscriber goal_sub;
    rosbag::Bag bag;
    bool write_to_bag = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_recorder");
    RosbagRecorder recorder;
    recorder.spin();
    return 0;
}
