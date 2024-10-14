#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>
#include <std_msgs/msg/string.hpp>
#include "bb_blackbox.hpp"
#include <rcl_interfaces/msg/log.hpp>

namespace blackbox
{

/// @brief ログの重要度を指定する
enum log_type_t{
    ERR=rcl_interfaces::msg::Log::ERROR,            // 常にレコード、Debug modeはSTDOUT
    WARN=rcl_interfaces::msg::Log::WARN,            // 常にレコード、Debug modeはSTDOUT
    INFO=rcl_interfaces::msg::Log::INFO,            // 常にレコード、 Debug modeはSTDOUT
    DEBUG=rcl_interfaces::msg::Log::DEBUG,          // Debug modeのときだけレコード + STDOUT
    LIB_INFO=rcl_interfaces::msg::Log::INFO+1,      // 常にレコード
    LIB_DEBUG=rcl_interfaces::msg::Log::DEBUG+1     // Debug modeのときだけレコード
};

class LogRecorder : BlackBoxWriter<rcl_interfaces::msg::Log>
{
public:
    class Logger
    {
    private:
        LogRecorder*    _handle = nullptr;
        log_type_t      _log_type;
        std::string     _tag_name;
        bool            _is_enable = true;

    public:
        Logger(void){}

        void init(LogRecorder* handle, log_type_t log_type, std::string tag_name){
            _handle = handle;
            _log_type = log_type;
            _tag_name = tag_name;

            if(_handle->_bb->_bb_debug_mode == debug_mode_t::RELEASE)
            {
                _is_enable = (_log_type != log_type_t::DEBUG);
            }else{
                _is_enable = true;
            }
        }


        static bool is_enable(Logger* obj){
            return (obj != NULL && obj->_is_enable && obj->_handle != NULL);
        }
        

        static void log(Logger* obj, const char* file, const char* func, size_t line, std::string str);
        static void log(Logger* obj, const char* file, const char* func, size_t line, const char* fmt, ...);

        static bool is_enable(Logger& obj){
            return is_enable(&obj);
        }

        template <class... Args>
        static void log(Logger& obj, Args... args){
            log(&obj, args...);
        }
    };

private:
    rclcpp::Node* _node = nullptr;
    blackbox::BlackBox* _bb = nullptr;

    void record(const rcl_interfaces::msg::Log* msg){
        BlackBoxWriter<rcl_interfaces::msg::Log>::write(*msg, msg->stamp);
    }

public:
    template <typename T>
    LogRecorder(T* node)
    {
        _node = static_cast<rclcpp::Node*>(node);
        _bb = static_cast<BlackBox*>(node);
        
        std::string ns = node->get_namespace();
        if(ns.size() != 1){
            ns += '/';
        }
        this->BlackBoxWriter_cons(_bb, "/tagger" + ns + node->get_name(), 0, rclcpp::RosoutQoS());
    }
};

using Logger = LogRecorder::Logger;


}

#define TAGGER(obj, ...) if(blackbox::Logger::is_enable(obj)){blackbox::Logger::log(obj, basename(__FILE__), __func__, __LINE__, __VA_ARGS__);}