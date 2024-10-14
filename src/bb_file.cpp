#include "../include/blackbox/bb_file.hpp"

#include <mutex>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <rclcpp/rclcpp.hpp>

namespace blackbox::tl_file
{

static std::mutex           g_locker;

volatile static bool        g_is_initialized = false;
static std::string          g_start_tim = "";

static FILE*                g_tl_info_fp = NULL;
static FILE*                g_err_fp = NULL;


static std::string tim_str()
{
    char header[128];
    struct timespec now;
    timespec_get(&now, TIME_UTC);

    int len = snprintf(header, 128, "%ld_%03ld",
            now.tv_sec,
            now.tv_nsec / 1000000
        );

    if(len < 0 )
    {
        len = 0;
        header[0] = '\0';
    }

    return header;
}

static void log_file(FILE* fp, const char* level, std::string msg)
{
    char header[128];
    struct timespec now;
    timespec_get(&now, TIME_UTC);

    int len = snprintf(header, 128, "[%4s] [%ld.%09ld] : ",
            level,
            now.tv_sec,
            now.tv_nsec
        );

    if(len < 0 )
    {
        len = 0;
        header[0] = '\0';
    }

    msg.insert(0, header, len);
    msg.append("\n");
    fputs(msg.c_str(), fp);
}


void init(void)
{
    g_locker.lock();
    if(g_is_initialized == false)
    {
        g_start_tim = tim_str();

        struct stat stat_buf;
        if(stat(TAGLOG_BASE_DIR, &stat_buf) != 0)
        {
            printf("[BlackBox] Stat Error! errno(\"%s\")\n", strerror(errno));

            int res = mkdir(TAGLOG_BASE_DIR,
                    S_IRUSR | S_IWUSR | S_IXUSR |           //ユーザーの権限（読み込み、書き込み、実行）
                    S_IRGRP | S_IWGRP | S_IXGRP |           //グループの権限（読み込み、書き込み、実行）
                    S_IROTH | S_IWOTH | S_IXOTH);           //その他の権限（読み込み、書き込み、実行）

            if(res != 0)
            {
                printf("[BlackBox] Mkdir Error! errno(\"%s\")\n", strerror(errno));
            }else{
                printf("[BlackBox] Create Directory.\n");
            }
            fflush(stdout);
        }

        std::string tl_log_path = TAGLOG_BASE_DIR"/tl_log.txt";
        g_tl_info_fp = fopen(tl_log_path.c_str(), "a");
        if(g_tl_info_fp == NULL)
        {
            g_tl_info_fp = stdout;

            log_file(stdout, "ERR", "Fopen Failed! \"" + tl_log_path + "\" errno(\"" + (std::string)strerror(errno));
            fflush(stdout);
        }
        else
        {
            setlinebuf(g_tl_info_fp);
        }

        std::string err_path = TAGLOG_BASE_DIR"/error.txt";
        g_err_fp = fopen(err_path.c_str(), "a");
        if(g_err_fp == NULL)
        {
            g_err_fp = stdout;

            log_file(g_tl_info_fp, "ERR", "Fopen Failed! \"" + err_path + "\" errno(\"" + (std::string)strerror(errno));
            fflush(g_tl_info_fp);
        }
        else
        {
            setlinebuf(g_err_fp);
            log_file(g_tl_info_fp, "INFO", "Open \"" + err_path + "\"");
        }
        g_is_initialized = true;
    }
    g_locker.unlock();
}


void err_file_out(std::string str)
{
    g_locker.lock();
    if(g_is_initialized)
    {
        str += "\n";
        fputs(str.c_str(), g_err_fp);
    }
    g_locker.unlock();
}


void err_file_out(std::string ns, std::string node, std::string msg)
{
    g_locker.lock();
    if(g_is_initialized)
    {
        err_file_out("(" + ns + "." + node + ") " + msg);
    }
    g_locker.unlock();
}


void create_node(std::string ns, std::string node)
{
    g_locker.lock();
    if(g_is_initialized)
    {
        std::string dir_path;
        if(ns.size() <= 1)
        {
            dir_path = TAGLOG_LOG_DIR(node, g_start_tim);
        }else{
            dir_path = TAGLOG_LOG_DIR_WITH_NS(ns, node, g_start_tim);
        }
        
        log_file(g_tl_info_fp, "INFO", "Mkdir \"" + dir_path + "\"");

        struct stat stat_buf;
        if(stat(dir_path.c_str(), &stat_buf) != 0)
        {
            int res = mkdir(dir_path.c_str(),
                    S_IRUSR | S_IWUSR | S_IXUSR |           //ユーザーの権限（読み込み、書き込み、実行）
                    S_IRGRP | S_IWGRP | S_IXGRP |           //グループの権限（読み込み、書き込み、実行）
                    S_IROTH | S_IWOTH | S_IXOTH);           //その他の権限（読み込み、書き込み、実行）

            if(res != 0)
            {
                log_file(g_tl_info_fp, "ERR", "Mkdir Error! errno(\"" + (std::string)strerror(errno) +"\")\n");
            }
            fflush(stdout);
        }
        else
        {
            log_file(g_tl_info_fp, "ERR", "Already exists \"" + dir_path + "\"");
        }
    }
    g_locker.unlock();
}

std::string get_rosbag_path(std::string ns, std::string node, std::string rosbag_name)
{
    if(ns.size() == 0) ns = "nameless";

    std::string result_path = "";

    g_locker.lock();
    if(g_is_initialized)
    {
        std::string path;
        if(ns.size() <= 1)
        {
            path = TAGLOG_LOG_DIR(node, g_start_tim);
        }else{
            path = TAGLOG_LOG_DIR_WITH_NS(ns, node, g_start_tim);
        }
        
        path += "/" + rosbag_name;
        
        struct stat stat_buf;
        if(stat(path.c_str(), &stat_buf) != 0)
        {
            result_path = path;
            log_file(g_tl_info_fp, "INFO", "[BAG_PATH] Valid path " + path);
        }
        else
        {
            log_file(g_tl_info_fp, "ERR", "[BAG_PATH] Invalid path \"" + path + "\" errno(\"" + (std::string)strerror(errno));

            path += "_tmp";
            struct stat stat_buf;
            if(stat(path.c_str(), &stat_buf) != 0)
            {
                result_path = path;
                log_file(g_tl_info_fp, "INFO", "[BAG_PATH] Valid path " + path);
            }else{
                log_file(g_tl_info_fp, "ERR", "[BAG_PATH] Invalid exists \"" + path + "\" errno(\"" + (std::string)strerror(errno));
            }
        }
    }
    g_locker.unlock();
    return result_path;
}

// std::string node_logger(std::string ns, std::string node)
// {
    
// }

}