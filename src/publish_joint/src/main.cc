#include "JointManager.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <cmath>
#include <nav_msgs/Path.h>
#include "publish_joint/Status.h" // 替换为你的包名和消息文件名
#include <std_msgs/Header.h>

using namespace std;

bool send_flag = false; // 全局标志位

class PosePublisher
{
public:
    PosePublisher()
    {
        // 初始化发布者
        pub = n.advertise<geometry_msgs::PoseStamped>("pose_publisher", 100);
        PathPublisher = n.advertise<nav_msgs::Path>("Path", 100);
        status_pub = n.advertise<publish_joint::Status>("status_topic", 1000);
        path_msg.poses.clear(); // 将当前位姿添加到轨迹中
    }

    void publishPose(JointManager &joint, JointManager::Status &status)
    {
        if (send_flag)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now(); // 设置时间戳
            pose_msg.header.frame_id = "map";         // 根据需要设置合适的坐标系
            float r = 0.42;

            pose_msg.pose.position.x = (status.mWalk / 2000. - (r * cos(status.mYaw))) - 23.9149;
            pose_msg.pose.position.y = -(status.mStretch / 1000. + 0.114);
            pose_msg.pose.position.z = (r * sin(status.mYaw)); // Z位置，根据实际需要设置
            std::cout << "x : " << (status.mWalk / 2000. + (r * cos(status.mYaw))) - 23.9149 << std::endl;

            Eigen::Matrix4f Tcw = JointManager::ToTcw(status);
            Eigen::Matrix3f rotation = Tcw.block<3, 3>(0, 0); // 获取3x3的旋转矩阵部分
            Eigen::Quaternionf quaternion(rotation);          // 从旋转矩阵构造四元数

            // std::cout << " Tcw: \n"
            //           << Tcw << std::endl;
            // 设置方向
            pose_msg.pose.orientation.x = quaternion.x();
            pose_msg.pose.orientation.y = quaternion.y();
            pose_msg.pose.orientation.z = quaternion.z();
            pose_msg.pose.orientation.w = quaternion.w();

            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = "map";
            path_msg.poses.push_back(pose_msg); // 将当前位姿添加到轨迹中
            PathPublisher.publish(path_msg);    // 发布轨迹

            pub.publish(pose_msg);

            publish_joint::Status msg;
            msg.header.stamp = ros::Time::now(); // 设置当前时间
            msg.mWalk = status.mWalk;
            msg.mStretch = status.mStretch;
            msg.mYaw = status.mYaw;
            msg.mPitch = status.mPitch;

            status_pub.publish(msg); // 发布消息
            // std::cout << "get in publish " << std::endl;
            send_flag = false; // Reset the flag
        }
    }
public:
    nav_msgs::Path path_msg;

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Publisher PathPublisher;
    ros::Subscriber sub;
    ros::Publisher status_pub;
};

int main(int argc, char **argv)
{

    cv::FileStorage configs("/usr/local/project/keystar/ROS_WS/src/publish_joint/config/auto_disploy.yaml", cv::FileStorage::READ);

    if (!configs.isOpened())
    {
        spdlog::error("Fail to open config file at {}", "../config/auto_disploy.yaml");
        exit(-1);
    }

    kr::InitLog("kr_logger", "./slam_log");

    // JointManager joints("tcp://10.30.1.135:1883", "joint_manager", 1);
    JointManager joints(configs["joints"]);

    // joints.InitMQTT();

    JointManager::Status status;

    ros::init(argc, argv, "pose_publisher_node");

    PosePublisher *posePublisher = new PosePublisher();

    ros::Rate loop_rate(10); // 设置循环频率为10Hz


    std::this_thread::sleep_for(std::chrono::seconds(5));
    posePublisher->path_msg.poses.clear();
    
    while (ros::ok())
    {

        ros::spinOnce();

        send_flag = joints.GetStatus(status, false);

        spdlog::info("status: walk[{}] stretch[{}] pitch[{}] yaw[{}]", status.mWalk, status.mStretch, status.mPitch, status.mYaw);

        posePublisher->publishPose(joints, status); // 检查标志位并可能发布消息
        // 行走控制(阻塞等待消息发送)
        // joints.Execute(JointManager::WalkJoint, status.mWalk++, true);
        // // 伸缩控制(非阻塞)
        // joints.Execute(JointManager::StretchJoint, status.mStretch++);
        // // 俯仰控制(非阻塞)
        // joints.Execute(JointManager::PitchJoint, status.mPitch++, false);

        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }

    spdlog::shutdown();
    return 0;
}
