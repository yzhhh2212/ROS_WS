#include <functional>

#include <readline/readline.h>  
#include <readline/history.h>

#include <mutex>

#include "tools/kr_cli.hpp"

// 命令支持单词个数的最大值
#define MAX_WORD_NUM    16
// 命令支持单个单词字母数最大值
#define MAX_CHAR_NUM    32


namespace kr {

/**
 * @brief 命令管理器是通过树结构存储所有命令
 * 1. 树中的节点除了叶节点存储的是命令的关键词,以及下一节点;
 * 2. 叶节点存储命令的执行函数;
*/
struct Node {
    
    Node() {}

    Node(CliHandlerType f): func(f) {}

    Node(const std::string& w) : word(w) {}

    // 节点对应命令关键词
    std::string word;
    // 命令的回调函数
    CliHandlerType func = nullptr;
    // 存储下一个节点的数组
    std::vector<Node> next;
};


static class {

public:

    /**
     * @brief 对字符串匹配已注册的命令行
     * @param text[in]: 待匹配的字符串指针
     * @param end[in]: 待匹配的字符串长度
     * @param checkAll[in]: true-整个字符串匹配才返回匹配成功 false-返回匹配失败的上一个结点
     * @return: 匹配到的关键词节点
    */
    const Node* Match(const char* text, size_t end, bool checkAll)
    {
        std::unique_lock<std::mutex> lock(mutex);

        const Node* pNode = &root;

        for(size_t offset = 0; offset < end; ) {

            if(text[offset] == ' ') {
                offset++;
                continue;
            }

            // 提取命输入行的关键词
            size_t charCnt = 0;
            char word[MAX_CHAR_NUM];
            while(text[offset] != ' ' && offset < end) {
                word[charCnt++] = text[offset++];
            }
            word[charCnt] = '\0';
            size_t len = strlen(word);

            // 匹配已注册的关键词
            bool matched = false;
            for(const auto& child: pNode->next) {
            
                if(!strncmp(word, child.word.c_str(), std::max(len, child.word.size()) )) {
                    // std::cout << "match sub key word [" << child.word << "]" << std::endl;
                    matched = true;
                    pNode = &child;
                    break;
                }
            }

            // if(!matched)
            //     return nullptr;
            if(!matched) {
                if(checkAll)
                    return nullptr;
                else
                    return pNode;
            }

        }

        return pNode;
    }

    /**
     * @brief 尝试执行输入栏的指令
     * 先执行命令匹配 匹配成功后将输入传给用户函数
     * @param line[in]: 输入的字符串指针
     * @return: true- 成功执行
     *          false-未找到匹配命令
    */
    bool TryExecute(const char* line)
    {
        size_t len = strlen(line);
        const Node* pNode = Match(line, len, false);

        for(auto& next: pNode->next) {
            
            if(next.func) {

                char buffer[MAX_CHAR_NUM*MAX_CHAR_NUM];
                char* argv[MAX_WORD_NUM];
                
                memcpy(buffer, line, len);
                buffer[len] = '\0';

                size_t cnt = 0;
                for(size_t i = 0; i < len; i++) {
                    if(buffer[i] == ' ') {
                        buffer[i] = '\0';
                        continue;
                    }
                    
                    argv[cnt++] = buffer + i;
                    while(buffer[i] != ' ' && i < len)
                        i++;

                    buffer[i] = '\0';
                }

                next.func(cnt, argv);
                return true;
            }
        }

        std::cout << "Input error! Command [" << line << "] not found." << std::endl; 
        return false;
    }

    /**
     * @brief 更新候选的节点,主要用于Readline的自动补全
     * @param pNode[in]: 当前的树节点
    */
    void UpdateCandidates(const Node* pNode)
    {
        candidates.clear();
        
        std::unique_lock<std::mutex> lock(mutex);

        for(auto& child: pNode->next) {
            if(!child.word.empty())
                candidates.push_back(child.word);
        }
    }

    /**
     * @brief 拷贝候选的节点,Readline的生成自动补全关键词时调用
    */
    const std::vector<std::string> GetCandidates(void)
    {
        return candidates;
    }

    /**
     * @brief 注册命令:将命令插入至命令树中,并生成对应叶子结点(用户提供的回调函数)
     * @param cmd[in]: 上层用户提供的命令结构体
    */
    bool RegisterCmd(const Command& cmd)
    {
        std::unique_lock<std::mutex> lock(mutex);
        Node* pNode = &root;

        auto itWord = cmd.mKeywords.begin();
        for( ; itWord != cmd.mKeywords.end(); itWord++) {

            if(itWord->empty()) {
                cmd.Show();
                std::cout << ". Format of command wrong, fail to register!" << std::endl;
                return false;
            }

            bool matched = false;
            for(auto& child: pNode->next) {

                if(child.word == *itWord) {
                    pNode = &child;
                    matched = true;
                    break;
                }
            }

            if(!matched)
                break;
        }

        if(itWord == cmd.mKeywords.end()) {
            cmd.Show();
            std::cout << " already register!" << std::endl;
            return false;
        }

        for( ; itWord != cmd.mKeywords.end(); itWord++) {
            size_t size = pNode->next.size();
            pNode->next.push_back(Node(*itWord));
            pNode = &pNode->next.at(size);
        }

        // finally, put handler into a empty node
        pNode->next.push_back(Node(cmd.mFunc));

        std::cout << "Command [";
        cmd.Show();
        std::cout << "] registered." << std::endl;
        return true;

    }

private:

    // 命令树的根节点
    Node root;
    std::mutex mutex;

    // 单次触发记录的临时变量
    //  其实可以不把candidate放在这个管理器中,但是Readline的库很多都是通过
    //  回调执行,使用全局的变量比较方便,所以就把这个candidate放在Manager中
    std::vector<std::string> candidates;
} manager;

/**
 * @brief 由Readline的rl_completion_matches内部调用
 * 主要功能是自动补全时返回可用的候选关键词
 * 每匹配成功一次就需要返回一次
 * 当所有候选的关键词都遍历完最后要返回NULL
*/
static char* Generator(const char *text, int state)
{
    static size_t index, len;

    if(!state) {
        index = 0;
        len  = strlen(text);
    }

    auto& candidates = manager.GetCandidates();

    for(; index < candidates.size(); ) {

        if(!strncmp(candidates[index].c_str(), text, len)) {
            return strdup(candidates[index++].c_str());
        }

        index++;
    }

    return NULL;
}


// 自定义补全函数  
static char** CustomCompletion(const char *text, int start, int end) 
{
    // 1. 对非自动补全部分进行匹配
    const Node* pNode = manager.Match(rl_line_buffer, start, true);
    if(!pNode)
        return nullptr;
    
    // 2. 待估计部分生成候选的单词
    manager.UpdateCandidates(pNode);

    auto& candidates = manager.GetCandidates();

    return rl_completion_matches(text, Generator);
}

void RegisterCmd(const Command& cmd)
{
    manager.RegisterCmd(cmd);
}


void LoopReadline(const std::string& name)
{
    static bool running = false;
    
    if(running) {
        std::cout << "cli is already running, do not create again!" << std::endl;
        return;
    }

    running = true;

    rl_readline_name = name.c_str();
    rl_attempted_completion_function = CustomCompletion;

    char* line, *start;
    while(line = readline("> ")) {

        start = line;
        while(*start == ' ')
            start++;

        if(*start) {
            // std::cout << "Entered: " << start << std::endl;
            manager.TryExecute(start);
            add_history(start);
        }

        free(line);
    }



}

}