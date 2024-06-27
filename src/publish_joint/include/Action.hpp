#ifndef _ACTION_HPP_
#define _ACTION_HPP_

#include "kr_utils.hpp"

class Action {
public:
    using Ptr = std::shared_ptr<Action>;

    enum State {
        Init = 1,
        // 等待请求应答
        WaitAck = 2,
        // 动作执行中
        Executing = 3,
        // 丢失心跳(下位机协议暂无)
        LostHeartBeat = 4,
        // 动作完成
        Finish = 5
    };

    enum Mode {
        // 停止
        Brake = 0,
        // 速度模式
        Speed = 1,
        // 位置模式
        Position = 2
    };

    // 动作类型
    enum Type {
        // 行走
        Walk = 1,
        // 伸缩
        Stretch = 2,
        // 偏航旋转
        YawRotate = 3,
        // 俯仰旋转
        PitchRotate = 4
    };

    Action(Type type, Mode mode, float value)
    : mId(id++), mMode(mode), mType(type)
    {
        if(mMode == Mode::Speed)
            mVelocity = value;
        else
            mPosition = value;

        mState = State::Init;

        spdlog::info("action id {}", mId);
    }

    static long unsigned int id;
    static std::map<Action::State, string> nameOfActionState;
    static std::map<Action::Mode, string> nameOfActionMode;
    static std::map<Action::Type, string> nameOfActionType;
    

    Action::State GetState(void) { return mState; }

    void SetState(State state) 
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mState = state;
        mStateCV.notify_all();
    }

    bool WaitToState(State state, int timeout = 0);

    std::string Info(void);

    const long unsigned int mId;
    Mode mMode;
    Type mType;

    float mPosition;
    float mVelocity;

private:
    // 方便上层使用时根据条件变量及时获取State更新
    // mMutex 只保护State
    std::mutex mMutex;
    // mStateCV 在State发生变化事会被通知
    std::condition_variable mStateCV;
    State mState;
};

#endif