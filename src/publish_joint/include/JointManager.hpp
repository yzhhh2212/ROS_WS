#ifndef _JOINT_MANAGER_HPP_
#define _JOINT_MANAGER_HPP_

#include "kr_utils.hpp"
#include "Action.hpp"

#include "mqtt/client.h"
#include "mqtt/async_client.h"



#include "rapidjson/document.h"
#include "rapidjson/writer.h" 
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

class JointCallback;

/**
 * @brief 关节管理类,管理机器人以下关节 
 *      1.行走关节; 2.伸缩关节; 3.俯仰关节.
 *  通过机器人的对外提供的MQTT协议封装以下功能
 *      1.电机里程查询; 2.关节控制.
*/
class JointManager {

public:
    friend JointCallback;
    using Ptr = shared_ptr<JointManager>;

    // 关节类型
    enum JointType {
        // 行走关节
        WalkJoint = 1,
        // 伸缩关节
        StretchJoint = 2,
        // 偏航关节
        YawJoint = 3,
        // 俯仰关节
        PitchJoint = 4
    };

    struct Status {

        Status(float walk, float stretch, float yaw, float pitch)
        : mWalk(walk), mStretch(stretch), mYaw(yaw), mPitch(pitch)
        {}

        Status(void) {}

        // unit: mm
        float mWalk;
        // unit: mm
        float mStretch;
        // unit: rad
        float mYaw;
        // unit: rad
        float mPitch;
    };

    /**
     * @brief 计算 底座坐标系 到 世界坐标系 的变换关系
     * @param x[in]: 底座x坐标
     * @param y[in]: 底座y坐标
    */
    static Eigen::Matrix4f ToTwb(float x, float y);

    /**
     * @brief 计算 相机坐标系 到 底座坐标系 的变换关系
     * @param yaw[in]: 底座的偏航角
     * @param pitch[in]: 相机臂的俯仰角
     * @note 需要用到机器人的一些结构几何参数
    */
    static Eigen::Matrix4f ToTbc(float yaw, float pitch);

    static Eigen::Matrix4f ToTwc(float x, float y, float yaw, float pitch)
    {
        return (ToTwb(x, y) * ToTbc(yaw, pitch));
    }

    static Eigen::Matrix4f ToTwc(const Status& status)
    {
        return ToTwc(status.mWalk, status.mStretch, - status.mYaw, status.mPitch);
    }

    static Eigen::Matrix4f ToTcw(const Status& status)
    {
        Eigen::Matrix4f T = ToTwc(status);
        Eigen::Matrix3f R = T.block<3,3>(0,0);
        Eigen::Vector3f t = T.block<3,1>(0,3);
        T.block<3,3>(0,0) =  R.transpose();
        T.block<3,1>(0,3) = -R.transpose()*t;

        return T;
    }

    /**
     * @brief 从文件中加载配置参数并初始化mqtt客户端
     * @param configs[in]: 配置文件中的joints节点
    */
    JointManager(const cv::FileNode& configs);

    /**
     * @param server[in]: MQTT服务器地址
     * @param clientId[in]: MQTT客户端的名字
     * @param qos[in]: 消息服务等级
    */
    JointManager(const std::string& server, const std::string& clientId, int qos);

    /**
     * @brief 初始化MQTT:1)连接服务器 2)注册订阅消息的回调函数
    */
    bool InitMQTT(void);

    /**
     * @brief 关节控制请求接口
     * @param action[in]: 动作结构体 描述待执行动作的关节/电机/参数等
     * @param retry[in]: 如果未收到请求应答则重新执行
     * @param timeout[in]: 阻塞等待应答的超时时间 unit ms
    */
    bool RequestAction(Action::Ptr& action, int retry = 1, int timeout = 20);

    /**
     * @brief 获取所有关节状态数据
     * @param status: 关节状态数据的引用
     * @param update: true-向机器人请求更新 false-直接获取记录的历史值
    */
    bool GetStatus(Status& status, bool update = false);

    /**
     * @brief 获取指定关节状态数据
     * @param which: 指定关节的枚举值,对应 JointType
     * @return 该关节的状态值
    */
    float GetStatus(int which);

// 以下是向CLI用户提供的功能
public:
    void HandleShowStatus(int argc, char** argv);

    void HandleExecute(int argc, char** argv);

    void HandleBrake(int argc, char** argv);

private:
    // 对应协议中 功能/属性表 的枚举值
    enum ValueType {
        Direction   = 1,
        Position    = 2,
        Degree      = 3,
        Speed       = 4
    };

    enum Direction {
        Stop = 0,
        Positive = 1,
        Negative = 2
    };

    /**
     * 执行发送MQTT消息的操作。
     * 这个函数构建一个MQTT消息，并将其发布到指定的主题。
     *
     * @param topic     消息的主题
     * @param payload   消息内容
     * @param block     true-阻塞等待发送完成 false-直接返回
     * @param qos       消息的服务质量等级
     */
    void Publish(const string& topic, const string& payload, bool block, int qos);

    /**
     * 处理动作请求的应答
     * @param doc   json格式的消息内容
    */
    void ProcessActResponse(const rapidjson::Document& doc);

    /**
     * 处理动作结束后的反馈
     * @param doc   json格式的消息内容
    */
    void ProcessActFinish(const rapidjson::Document& doc);

    /**
     * 监听编码器状态发布服务器的线程
    */
    void OdomListen(void);

    // 底座中心 到 相机臂轴心 偏移 unit: mm
    static float lenObOa;
    // 相机臂轴心 到 相机光心 偏移 unit: mm
    static float lenOaOc;

    // 编码器实时状态发布 socket
    int mSock;
    std::thread mOdomListenThread;

    // The MQTT client
    shared_ptr<mqtt::async_client> mpClient;
    // Options to use if we need to reconnect
    mqtt::connect_options mConnOpts;
    // subscription callback
    std::shared_ptr<JointCallback> mpCallback;
    // 消息服务质量
    int mQos;

    // 客户端发布的话题 如: KR/GGXJ/000E3D73081A
    string mPubTopic;
    // 发布控制的子话题 如: KR/GGXJ/operation
    string mControlSubTopic;

    // 订阅机器人的话题 如: KR/GGXJ/operation 
    vector<string> mvRobotTopic;

    // 以下是协议相关的
    const size_t DEVICE_ID_OFFSET = 500;
    string mRobotId;

    // 关节状态
    std::mutex mMutex;
    Status mStatus;

    // 关节的零点
    Status mZeros;

    // 执行中的动作集合
    std::mutex mMutexAction;
    std::set<Action::Ptr> msActions;
};






#endif