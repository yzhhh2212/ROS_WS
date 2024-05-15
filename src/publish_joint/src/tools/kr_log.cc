#include "tools/kr_log.hpp"
#include "tools/kr_cli.hpp"

namespace kr {

static std::pair<spdlog::level::level_enum, std::string> gSpdlogLevelInfo[7] = {
    {spdlog::level::trace,      "trace"},
    {spdlog::level::debug,      "debug"},
    {spdlog::level::info,       "info"},
    {spdlog::level::warn,       "warn"},
    {spdlog::level::err,        "err"},
    {spdlog::level::critical,   "critical"},
    {spdlog::level::off,        "off"},
};

/**
 * @brief 改变控制台的输出等级
 * Usage: logger set level [l]
*/
static void HandleLoggerSetLevel(int argc, char** argv)
{
    if(argc < 4) {
        spdlog::warn("Usage: logger set level [l]\n"
            "0-{}\n1-{}\n2-{}\n3-{}\n4-{}\n5-{}\n6-{}", 
            gSpdlogLevelInfo[0].second, gSpdlogLevelInfo[1].second,
            gSpdlogLevelInfo[2].second, gSpdlogLevelInfo[3].second,
            gSpdlogLevelInfo[4].second, gSpdlogLevelInfo[5].second,
            gSpdlogLevelInfo[6].second);
        return;
    }

    int level = atoi(argv[3]);
    if(level < 0 || level > 6) {
        spdlog::warn("spdlog does not support level [{}]", level);
        return;
    }

    spdlog::default_logger()->sinks()[0]->set_level(gSpdlogLevelInfo[level].first);
    spdlog::info("logger set level [{}]", gSpdlogLevelInfo[level].second);
}


void InitLog(const std::string& name, const std::string& file)
{
    static std::shared_ptr<spdlog::logger> logger;

    logger = spdlog::get(name);
    
    if(!logger) {
        // 1. console
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("[%^%l%$] %v");
        console_sink->set_level(spdlog::level::debug);

        // 2. log file
        auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(file, 0, 0);
        // file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
        file_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
        file_sink->set_level(spdlog::level::info);
        
        // 组合上面2个sink
        logger = std::make_shared<spdlog::logger>(name, spdlog::sinks_init_list{console_sink, file_sink});

        spdlog::register_logger(logger);
        // 下面的level决定所有sink的下限,即如果连下面的等级都不满足,是绝对不会触发上面2个sink
        spdlog::set_level(spdlog::level::trace);
        spdlog::flush_on(spdlog::level::debug);
        spdlog::set_default_logger(logger);
    } else {
        spdlog::warn("{} already exist!", name);
    }

    kr::RegisterCmd(kr::Command(
        std::vector<std::string>{"logger", "set", "level"}, HandleLoggerSetLevel));
}

}