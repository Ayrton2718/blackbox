#include "../include/blackbox/bb_logger.hpp"

namespace blackbox
{

void LogRecorder::Logger::log(Logger* obj, const char* file, const char* func, size_t line, std::string str)
{
    rcl_interfaces::msg::Log msg;
    msg.stamp = obj->_handle->get_bb_tim();
    msg.name = obj->_tag_name;
    msg.level = (static_cast<uint8_t>(obj->_log_type) / 10) * 10;
    msg.file = file;
    msg.function = func;
    msg.line = line;
    msg.msg = str;
    obj->_handle->write(msg);

    if(obj->_handle->_bb->_bb_debug_mode == debug_mode_t::DEBUG)
    {
        switch(obj->_log_type)
        {
        case log_type_t::ERR:
            RCLCPP_ERROR(obj->_handle->_node->get_logger(), str.c_str());
            break;

        case log_type_t::WARN:
            RCLCPP_WARN(obj->_handle->_node->get_logger(), str.c_str());
            break;

        case log_type_t::INFO:
        case log_type_t::DEBUG:
            RCLCPP_INFO(obj->_handle->_node->get_logger(), str.c_str());
            break;

        default:
            break;
        }
    }
}


void LogRecorder::Logger::log(Logger* obj, const char* file, const char* func, size_t line, const char* fmt, ...)
{
    std::string str;
    va_list ap;
    va_list ap_copy;
    va_copy(ap_copy, ap);

    va_start(ap, fmt);
    str.resize(1024);
    int len = vsnprintf(&str[0], 1024, fmt, ap);
    va_end(ap);

    if(len < 1024)
    {
        str.resize(len + 1);
    }else{
        str.resize(len + 1);  // need space for NUL

        va_start(ap_copy, fmt);
        vsnprintf(&str[0], len + 1, fmt, ap_copy);
        va_end(ap_copy);
    }

    Logger::log(obj, file, func, line, str);
}

}


