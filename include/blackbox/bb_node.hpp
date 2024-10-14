#pragma once

#include <rclcpp/rclcpp.hpp>
#include "bb_blackbox.hpp"
#include "bb_diagnostics.hpp"
#include "bb_logger.hpp"
#include "bb_param.hpp"

namespace blackbox
{


class BlackBoxNode : public rclcpp::Node, public BlackBox, public DiagnosticUpdater, public LogRecorder, public ParamParser
{
public:
    BlackBoxNode(debug_mode_t debug_mode, const std::string& node_name, const std::string& name_space = "", 
                const rclcpp::NodeOptions &options=rclcpp::NodeOptions(), bool param_overwrite=true,
                storage_profile_t storage_profile=storage_profile_t::zstd_fast, uint64_t max_cache_size=4096)
        :   rclcpp::Node(node_name, name_space, options), 
            BlackBox(this, debug_mode, "blackbox", storage_profile, max_cache_size),
            DiagnosticUpdater(this, node_name),
            LogRecorder(this),
            ParamParser(this, param_overwrite)
    {
    }
};

}