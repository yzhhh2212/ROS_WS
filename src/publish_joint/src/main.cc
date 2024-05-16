#include "JointManager.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

bool send_flag = false; // 全局标志位

class PosePublisher
{
public:
    PosePublisher()
    {
        // 初始化发布者
        pub = n.advertise<geometry_msgs::PoseStamped>("pose_publisher", 10);
    }

    void publishPose(JointManager &joint, JointManager::Status &status)
    {
        if (send_flag)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = ros::Time::now(); // 设置时间戳
            pose_msg.header.frame_id = "map";         // 根据需要设置合适的坐标系

            pose_msg.pose.position.x = status.mWalk / 1000.;
            pose_msg.pose.position.y = status.mStretch / 1000.;
            pose_msg.pose.position.z = 0.; // Z位置，根据实际需要设置

            Eigen::Matrix4f Tcw = JointManager::ToTcw(status);
            Eigen::Matrix3f rotation = Tcw.block<3, 3>(0, 0); // 获取3x3的旋转矩阵部分
            Eigen::Quaternionf quaternion(rotation);          // 从旋转矩阵构造四元数

            std::cout << " Tcw: \n" << Tcw <<std::endl;
            // 设置方向
            pose_msg.pose.orientation.x = quaternion.x();
            pose_msg.pose.orientation.y = quaternion.y();
            pose_msg.pose.orientation.z = quaternion.z();
            pose_msg.pose.orientation.w = quaternion.w();

            pub.publish(pose_msg);
            send_flag = false; // Reset the flag
        }
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};

int main(int argc, char **argv)
{

    kr::InitLog("kr_logger", "./slam_log");

    JointManager joints("tcp://10.30.1.135:1883", "joint_manager", 1);

    joints.InitMQTT();

    JointManager::Status status;

    ros::init(argc, argv, "pose_publisher_node");

    PosePublisher *posePublisher = new PosePublisher();

    ros::Rate loop_rate(10); // 设置循环频率为10Hz

    while (ros::ok())
    {

        ros::spinOnce();
        send_flag = joints.GetStatus(status, true);

        spdlog::info("status: walk[{}] stretch[{}] pitch[{}]", status.mWalk, status.mStretch, status.mPitch);

        posePublisher->publishPose(joints, status); // 检查标志位并可能发布消息
        // 行走控制(阻塞等待消息发送)
        // joints.Execute(JointManager::WalkJoint, status.mWalk++, true);
        // // 伸缩控制(非阻塞)
        // joints.Execute(JointManager::StretchJoint, status.mStretch++);
        // // 俯仰控制(非阻塞)
        // joints.Execute(JointManager::PitchJoint, status.mPitch++, false);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    spdlog::shutdown();
    return 0;
}
