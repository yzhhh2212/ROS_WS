#include <pcl/pcl_macros.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h" 
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "JointManager.hpp"

// 底座中心 到 相机臂轴心 偏移 unit: mm default: 97.5mm
float JointManager::lenObOa = 0.0;
// 相机臂轴心 到 相机光心 偏移 unit: mm default: 69.2mm
float JointManager::lenOaOc = 0.0;

class ActionListener : public virtual mqtt::iaction_listener {

public:
    
    ActionListener(const string& name) : mName(name) {}

    void on_failure(const mqtt::token& tok) override
    {
        spdlog::error("{} failure", mName);
        if (tok.get_message_id() != 0)
            spdlog::error(" for token: [{}]", tok.get_message_id());
    }

    void on_success(const mqtt::token& tok) override
    {
        spdlog::info("{} success", mName);
        if (tok.get_message_id() != 0)
            spdlog::info(" for token: [{}]", tok.get_message_id());
        auto top = tok.get_topics();
        if (top && !top->empty())
            spdlog::info("\ttoken topic: '{}', ...", (*top)[0]);
    }

private:
    string mName;
};


class JointCallback : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener {
    

public:
    JointCallback(JointManager* pManager, int retry = 5): 
        mpManager(pManager), 
        mSubListener(pManager->mpClient->get_client_id()+"_Subscription"), 
        mRetryAttempts(retry)
    {}

private:
    void reconnect()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        spdlog::warn("Reconnecting...(attempt[{}])", mCntRetry);
        try {
            mpManager->mpClient->connect(mpManager->mConnOpts, nullptr, *this);
        }
        catch (const mqtt::exception& exc) {
            spdlog::error("Mqtt reconnect fail. Error: {}", exc.what());
			// exit(1);
		}
    }

    // Re-connection failure
    void on_failure(const mqtt::token& tok) override 
    {
        spdlog::warn("Connection attempt failed");
		if (++mCntRetry > mRetryAttempts) {
            spdlog::error("Mqtt reconnect fail, retry over [{}] times.", mRetryAttempts);
        }
		reconnect();
	}

    // (Re)connection success
	// Either this or connected() can be used for callbacks.
    void on_success(const mqtt::token& tok) override {}

    // (Re)connection success
	void connected(const std::string& cause) override 
    {
        size_t n = mpManager->mvRobotTopic.size();

        spdlog::info("Connection success");
        for(size_t i = 0; i < n; i++) {
            spdlog::info("Subscribing to topic [{}]\tusing Qos [{}]", mpManager->mvRobotTopic[i], mpManager->mQos);
        }

        if(n == 1) {
            mpManager->mpClient->subscribe(mpManager->mvRobotTopic[0], mpManager->mQos, nullptr, mSubListener);
        } else if(n > 1) {
            // 订阅多个话题时需构造 mqtt::string_collection
            auto topics = mqtt::string_collection::create(mpManager->mvRobotTopic);
            vector<int> qoses(n, mpManager->mQos);
            mpManager->mpClient->subscribe(topics, qoses, nullptr, mSubListener);
        }
	}

    // JointCallback for when the connection is lost.
	// This will initiate the attempt to manually reconnect.
	void connection_lost(const std::string& cause) override 
    {
        spdlog::warn("Connection lost");
		if (!cause.empty()) {
            spdlog::warn("\tcause: ", cause);
        }

        spdlog::warn("Reconnecting...");
		mCntRetry = 0;
		reconnect();
	}

    void delivery_complete(mqtt::delivery_token_ptr token) override {}

    // JointCallback for when a message arrives.
	void message_arrived(mqtt::const_message_ptr msg) override;

    // 上层提供的指针用于存放数据
    JointManager* mpManager;

    // An action listener to display the result of actions.
    ActionListener mSubListener;

    int mCntRetry;
    int mRetryAttempts;

};


Eigen::Matrix4f JointManager::ToTwb(float x, float y)
{
    Eigen::Matrix4f Twb = Eigen::Matrix4f::Identity();
    Twb(0, 3) = x;
    Twb(1, 3) = y;
    return Twb;
}

Eigen::Matrix4f JointManager::ToTbc(float yaw, float pitch)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY()));
    transform.translate(Eigen::Vector3f(JointManager::lenObOa, 0, 0));
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()));
    transform.translate(Eigen::Vector3f(0, 0, JointManager::lenOaOc));
    
    return transform.matrix();
}

JointManager::JointManager(const cv::FileNode& configs)
{
    mStatus.mWalk = 0.0;
    mStatus.mStretch = 0.0;
    mStatus.mPitch = 0.0;
    mStatus.mYaw = 0.0;
    
    // 加载协议的话题
    mRobotId = static_cast<string>(configs["robotId"]);

    const cv::FileNode& robotTopic = configs["robotTopic"];
    for(cv::FileNodeIterator it = robotTopic.begin(); it != robotTopic.end(); it++) {
        mvRobotTopic.push_back(static_cast<string>(*it));
        spdlog::info("Mqtt subscribe topic: [{}]", mvRobotTopic.back());
    }

    const cv::FileNode& subTopic = configs["subTopic"];
    mControlSubTopic = static_cast<string>(subTopic["control"]);

    mPubTopic = "KR/GGXJ/" + mRobotId;

    // 创建mqtt客户端
    const cv::FileNode& mqttServer = configs["mqttServer"];
    mpClient = std::make_shared<mqtt::async_client>(string(mqttServer["uri"]), "auto_disploy");

    mQos << mqttServer["qos"];
    if (mQos < 0 || mQos > 2) {
        spdlog::warn("Qos must be 0, 1, or 2, Invalid value,Set to 1");
        mQos = 1;
    }

    configs["zeros"]["walk"] >> mZeros.mWalk;
    configs["zeros"]["stretch"] >> mZeros.mStretch;
    mZeros.mYaw = DEG2RAD(static_cast<float>(configs["zeros"]["yaw"]));
    mZeros.mPitch = DEG2RAD(static_cast<float>(configs["zeros"]["pitch"]));

    configs["ObOa"] >> JointManager::lenObOa;
    configs["OaOc"] >> JointManager::lenOaOc;

    kr::RegisterCmd(kr::Command(
        vector<string>{"joints", "show", "status"},
        std::bind(&JointManager::HandleShowStatus, this, std::placeholders::_1, std::placeholders::_2)
    ));

    kr::RegisterCmd(kr::Command(
        vector<string>{"joints", "execute"},
        std::bind(&JointManager::HandleExecute, this, std::placeholders::_1, std::placeholders::_2)
    ));
}

JointManager::JointManager(const std::string& server, const std::string& clientId, int qos)
{
    mpClient = std::make_shared<mqtt::async_client>(server, clientId);

    if (qos < 0 || qos > 2) {
        spdlog::warn("Qos must be 0, 1, or 2, Invalid value,Set to 1");
        qos = 1;
    }
    mQos = qos;

    // 订阅的话题
    mvRobotTopic.push_back("KR/GGXJ/operation");
    mvRobotTopic.push_back("KR/GGXJ/reportmsg");

    mControlSubTopic = "KR/GGXJ/operation";
    mPubTopic = "KR/GGXJ/KRXJ04-V3-001";

    mRobotId = "KRXJ04-V3-001";
}

bool JointManager::InitMQTT()
{
    const char *LWT_PAYLOAD = "Last will and testament.";
    mConnOpts = mqtt::connect_options_builder()
                        .clean_session()
                        .will(mqtt::message("lwt", LWT_PAYLOAD, strlen(LWT_PAYLOAD), 1, false))
                        .finalize();


    mpCallback = std::make_shared<JointCallback>(this);
    mpClient->set_callback(*mpCallback);

    try {
        spdlog::info("Connecting to the MQTT server [{}]...", mpClient->get_server_uri());
        spdlog::info("Waiting for the connection...");
		mqtt::token_ptr tok = mpClient->connect(mConnOpts, nullptr, *mpCallback);
        // 如果没有完全连接成功,容易导致出错
        tok->wait();
        spdlog::info("  ...OK");
	}
	catch (const mqtt::exception& exc) {
        spdlog::error("ERROR: Unable to connect to MQTT server: [{} {}]", 
            mpClient->get_server_uri(), exc.get_error_str());
		return false;
	}

    return true;
}

void JointManager::Publish(const string& topic, const string& payload, bool block, int qos)
{
    if(!mpClient->is_connected()) {
        spdlog::warn("Publish fail. [{}] not connected.", mpClient->get_client_id());
        return;
    }

    auto msg = mqtt::make_message(topic, payload);
    msg->set_qos(qos);

    mqtt::delivery_token_ptr pubtok;

    pubtok = mpClient->publish(msg);
    if(block) {
        if(!pubtok->wait_for(1000))
            spdlog::warn("Message publish timeout. [{}]:[{}]", topic, payload);
    }

}

void JointCallback::message_arrived(mqtt::const_message_ptr msg) 
{
    string topic = msg->get_topic();

    rapidjson::Document doc;
    doc.Parse(msg->to_string().c_str());

    if(doc.HasParseError()) {
        spdlog::warn("Message [{}] parse err.", topic);
        return;
    }

    #if 0
    // 打印消息
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    spdlog::info("Get message:\n{}", buffer.GetString());
    #endif

    if(doc.HasMember("cmd")) {
        string cmd = doc["cmd"].GetString();
        if(cmd != "report_state" && cmd != "get_state" && cmd != "report")
            return;
    } else 
        return;

    if(doc.HasMember("id")) {
        string id = doc["id"].GetString();
        if(id != mpManager->mRobotId)
            return;
    }

    if(!doc.HasMember("data") || !doc["data"].IsArray())
        return;
    
    std::unique_lock<std::mutex> lock(mpManager->mMutex);

    rapidjson::Value& data = doc["data"].GetArray();
    for(size_t i = 0, n = data.Size(); i < n; i++) {

        size_t sn = std::stoi(data[i]["sn"].GetString());
        size_t type = std::stoi(data[i]["value_type"].GetString());
        
        if(sn == (JointManager::WalkJoint+mpManager->DEVICE_ID_OFFSET) && 
           type == JointManager::Position) {
            // 行走电机数据
            mpManager->mStatus.mWalk = std::stof(data[i]["value"].GetString());

        } else if(sn == (JointManager::StretchJoint+mpManager->DEVICE_ID_OFFSET) && 
           type == JointManager::Position) {
            // 伸缩电机数据
            mpManager->mStatus.mStretch = std::stof(data[i]["value"].GetString());

        } else if(sn == (JointManager::YawJoint+mpManager->DEVICE_ID_OFFSET) && 
           type == JointManager::Degree) {
            // 偏航电机数据
            float value = std::stof(data[i]["value"].GetString()) / 100.;
            mpManager->mStatus.mYaw = -(DEG2RAD(value-180.) );

        } else if(sn == (JointManager::PitchJoint+mpManager->DEVICE_ID_OFFSET) && 
           type == JointManager::Degree) {
            // 俯仰电机数据
            float value = std::stof(data[i]["value"].GetString()) / 100.;
            mpManager->mStatus.mPitch = (DEG2RAD(value - 270.) );
            std::cout << "修改前 ： " << value << std::endl;
            std::cout << "修改 ： " << mpManager->mStatus.mPitch << std::endl;

        }
    }
}


bool JointManager::GetStatus(Status& status, bool update)
{
    if(update) {
        rapidjson::Document doc;
        auto& allocator = doc.GetAllocator();
        doc.SetObject();

        doc.AddMember("topic", rapidjson::StringRef(mControlSubTopic.c_str()), allocator);
        doc.AddMember("type", "rep", allocator);
        doc.AddMember("cmd", "get_state", allocator);
        doc.AddMember("data", rapidjson::Value(rapidjson::kArrayType).Move(), allocator);

        rapidjson::Value& data = doc["data"].GetArray();
        data.PushBack(JointManager::WalkJoint + DEVICE_ID_OFFSET, allocator);
        data.PushBack(JointManager::StretchJoint + DEVICE_ID_OFFSET, allocator);
        data.PushBack(JointManager::YawJoint + DEVICE_ID_OFFSET, allocator);
        data.PushBack(JointManager::PitchJoint + DEVICE_ID_OFFSET, allocator);

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        doc.Accept(writer);

        Publish(mPubTopic, buffer.GetString(), true, mQos);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
        std::unique_lock<std::mutex> lock(mMutex);
        status = mStatus;
    }
    return true;
}

float JointManager::GetStatus(int which)
{
    std::unique_lock<std::mutex> lock(mMutex);

    if(which == WalkJoint) {
        return mStatus.mWalk;
    } else if( which == StretchJoint) {
        return mStatus.mStretch;
    } else if( which == YawJoint) {
        return mStatus.mYaw;
    } else if( which == PitchJoint) {
        return mStatus.mPitch;
    } else {
        return 0.0;
    }
}

void JointManager::Execute(int which, float value, bool block)
{
    if(which < 0 || which > PitchJoint)
        return;
    
    rapidjson::Document doc;
    auto& allocator = doc.GetAllocator();
    doc.SetObject();
    
    doc.AddMember("topic", rapidjson::StringRef(mControlSubTopic.c_str()), allocator);
    doc.AddMember("type", "req", allocator);
    doc.AddMember("cmd", "control", allocator);
    doc.AddMember("data", rapidjson::Value(rapidjson::kArrayType).Move(), allocator);

    rapidjson::Value& data = doc["data"].GetArray();
    data.PushBack(rapidjson::Value(rapidjson::kObjectType), allocator);
    rapidjson::Value& actionCmd = data[0];

    if(which == WalkJoint) {
        // 行走控制
        actionCmd.AddMember("sn", JointManager::WalkJoint + DEVICE_ID_OFFSET, allocator);
        actionCmd.AddMember("value_type", JointManager::Position, allocator);
        actionCmd.AddMember("value", rapidjson::Value(std::to_string(value).c_str(), allocator), allocator);
    } else if(which == StretchJoint) {
        // 伸缩控制
        actionCmd.AddMember("sn", JointManager::StretchJoint + DEVICE_ID_OFFSET, allocator);
        actionCmd.AddMember("value_type", JointManager::Position, allocator);
        actionCmd.AddMember("value", rapidjson::Value(std::to_string(value).c_str(), allocator), allocator);
    } else if(which == YawJoint) {
        // 偏航控制
        actionCmd.AddMember("sn", JointManager::YawJoint + DEVICE_ID_OFFSET, allocator);
        actionCmd.AddMember("value_type", JointManager::Degree, allocator);
        actionCmd.AddMember("value", rapidjson::Value(std::to_string(-(value + 180.)*100.).c_str(), allocator), allocator);
    } else if(which == PitchJoint) {
        // 俯仰控制
        actionCmd.AddMember("sn", JointManager::PitchJoint + DEVICE_ID_OFFSET, allocator);
        actionCmd.AddMember("value_type", JointManager::Degree, allocator);
        actionCmd.AddMember("value", rapidjson::Value(std::to_string((value + 270.)*100.).c_str(), allocator), allocator);
    }

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);

    Publish(mPubTopic, buffer.GetString(), block, mQos);
}


void JointManager::HandleShowStatus(int argc, char** argv)
{
    Status status;
    GetStatus(status, true);

    spdlog::info("walk [{:.2f}]mm stretch [{:.2f}]mm yaw[{:.2f}]degree pitch[{:.2f}]degree", 
        status.mWalk, status.mStretch, RAD2DEG(status.mYaw), RAD2DEG(status.mPitch));
    
    std::cout << "Twb\n" << ToTwb(status.mWalk, status.mStretch) 
              << "\nTbc\n" << ToTbc(status.mYaw, status.mPitch) 
              << "\nTwc\n" << ToTwc(status) << std::endl;
    

    std::cout << "Tcw\n" << ToTcw(status) << std::endl;
}

void JointManager::HandleExecute(int argc, char** argv)
{
    if(argc < 4) {
        spdlog::warn("Usage: joints execute [1|2|3|4] [position]\n"
        "\t1-walk; 2-stretch; 3-yaw; 4-pitch");
        return;
    }

    int which = std::stoi(argv[2]);
    float value = std::stof(argv[3]);
    
    Execute(which, value, true);
}