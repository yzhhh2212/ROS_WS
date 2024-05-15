#ifndef _KR_CLI_HPP_
#define _KR_CLI_HPP_

#include <iostream>
#include <vector>
#include <string>


namespace kr {

typedef std::function<void (int, char**)> CliHandlerType;

/**
 * @brief 注册命令需提供的 命令 结构体
*/
struct Command {

    Command() {}
    
    Command(const std::vector<std::string>& keywords, CliHandlerType func)
    : mKeywords(keywords), mFunc(func)
    {}

    void Show(void) const
    {
        for(const auto& word: mKeywords)
            std::cout << word << " ";
    }

    // 命令的关键词数组
    std::vector<std::string> mKeywords;
    // 命令的回调处理函数
    CliHandlerType mFunc;

};

/**
 * @brief 用于注册命令
 * @param cmd[in]: 提供上述的命令结构体
*/
void RegisterCmd(const Command& cmd);

/**
 * @brief 阻塞读取用户当前的输入
 * @param name[in]: 为Readline的初始化提供一个名字
*/
void LoopReadline(const std::string& name);


/**
 * 命令行使用方法如下:
 * 
 * void HelloWorld(int argc, char** argv)
 * {
 *      ...
 * }
 * 
 * int main()
 * {
 *      ...
 *      // 注册命令: 关键词为[hello world], 回调函数为HelloWorld
 *      kr::RegisterCmd(std::vector<std::string>{"hello", "world"}, HelloWorld));
 *      ...
 * 
 *      // 阻塞等待用户键入
 *      kr::LoopReadline("kr_cli");
 * 
 *      ...
 * }
*/

}


#endif