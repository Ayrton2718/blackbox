#pragma once

#include <rclcpp/rclcpp.hpp>
#include "bb_blackbox.hpp"
#include "bb_logger.hpp"
#include "bb_param.hpp"

namespace blackbox
{

class BlackBoxNode : public rclcpp::Node, public BlackBox, public LogRecorder, public ParamUpdater
{
public:
    BlackBoxNode(debug_mode_t debug_mode, const std::string& node_name, const std::string& name_space = "", 
                const rclcpp::NodeOptions &options=rclcpp::NodeOptions(), bool enable_update=false,
                storage_profile_t storage_profile=storage_profile_t::zstd_fast, uint64_t max_cache_size=1024*128)
        :   rclcpp::Node(node_name, name_space, options), 
            BlackBox(this, debug_mode, "blackbox", storage_profile, max_cache_size),
            LogRecorder(this), ParamUpdater(this, enable_update)
    {
    }
};

}