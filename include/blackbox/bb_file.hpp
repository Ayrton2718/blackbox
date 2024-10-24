#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

// ログファイルの出力先
#define TAGLOG_BASE_DIR  "/tmp/blackbox_log"
#define TAGLOG_LOG_DIR(node, tim) (TAGLOG_BASE_DIR"/" + node + "-" + tim)
#define TAGLOG_LOG_DIR_WITH_NS(ns, node, tim) (TAGLOG_BASE_DIR + ns + "-" + node + "-" + tim)

namespace blackbox::tl_file
{

void init(void);

void err_file_out(std::string str);
void err_file_out(std::string ns, std::string node, std::string msg);

void create_node(std::string ns, std::string node);
std::string get_rosbag_path(std::string ns, std::string node, std::string rosbag_name);

}