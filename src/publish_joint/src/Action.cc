#include "Action.hpp"

long unsigned int Action::id = 0;

std::map<Action::State, string> Action::nameOfActionState = {
    {State::Init, "Init"},
    {State::WaitAck, "WaitAck"},
    {State::Executing, "Executing"},
    {State::LostHeartBeat, "LostHeartBeat"},
    {State::Finish, "Finish"}
};

std::map<Action::Mode, string> Action::nameOfActionMode = {
    {Mode::Brake, "Brake"},
    {Mode::Speed, "Speed"},
    {Mode::Position, "Position"},
};

std::map<Action::Type, string> Action::nameOfActionType = {
    {Type::Walk, "Walk"},
    {Type::Stretch, "Stretch"},
    {Type::YawRotate, "YawRotate"},
    {Type::PitchRotate, "PitchRotate"}
};

bool Action::WaitToState(State state, int timeout)
{
    TicToc timer;
    std::unique_lock<std::mutex> lock(mMutex);

    if(timeout > 0) {
        while(mState != state && timeout > 0) {
            timeout -= timer.toc();
            mStateCV.wait_for(lock, std::chrono::milliseconds(timeout));
        }
    } else {
        while(mState != state) {
            mStateCV.wait(lock);
        }
    }

    if(mState != state) {
        spdlog::warn("Wait for action [{} {}] to [{}] timeout.", 
            nameOfActionType[mType], nameOfActionMode[mMode], nameOfActionState[mState]);
        return false;
    }

    return true;
}

std::string Action::Info(void)
{
    std::ostringstream oss;
    oss << "aciton [" 
        << mId << " "
        << nameOfActionType[mType] << " " 
        << nameOfActionMode[mMode] << " "
        << nameOfActionState[mState] << "]";
    return oss.str();
}