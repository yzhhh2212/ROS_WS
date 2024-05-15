#ifndef _KR_LOG_HPP_
#define _KR_LOG_HPP_

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"

namespace kr {

void InitLog(const std::string& name, const std::string& file);


}

#endif