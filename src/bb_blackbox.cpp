#include "../include/blackbox/bb_blackbox.hpp"
#include "../include/blackbox/bb_file.hpp"

#include <rosbag2_cpp/writer.hpp>

namespace blackbox
{

std::string BlackBox::storage_profile_toString(storage_profile_t config)
{
    switch (config) {
        case storage_profile_t::fastwrite:
            return "fastwrite";
        case storage_profile_t::zstd_fast:
            return "zstd_fast";
        case storage_profile_t::zstd_small:
            return "zstd_small";
        default:
            return "zstd_fast";
    }
}

BlackBox::BlackBox(rclcpp::Node* node, debug_mode_t debug_mode, std::string file_name, storage_profile_t storage_preset_profile, uint64_t max_cache_size) : _bb_debug_mode(debug_mode)
{    
    tl_file::init();
    tl_file::create_node(node->get_namespace(), node->get_name());
    std::string blackbox_path = tl_file::get_rosbag_path(node->get_namespace(), node->get_name(), file_name);
    if(blackbox_path == "")
    {
        return;
    }

    _clock = node->get_clock().get();
    auto start = std::chrono::system_clock::now();
    while((_clock->ros_time_is_active()) && (_clock->now().nanoseconds() == 0))
    {
        if(std::chrono::milliseconds(10000) < (std::chrono::system_clock::now() - start)){
            break;
        }
    }

    rosbag2_storage::StorageOptions options;
    options.uri = blackbox_path;
    options.storage_id = "mcap";
    options.storage_preset_profile = storage_profile_toString(storage_preset_profile);
    options.max_cache_size = max_cache_size;
    options.max_bagfile_duration = 0;
    options.max_bagfile_size = 0;
    options.snapshot_mode = false;
    options.storage_config_uri = "";

    _writer = std::make_unique<rosbag2_cpp::Writer>();
    _writer->open(options);
}


void BlackBox::create(std::string topic_name, std::string msg_type, const rclcpp::QoS& qos)
{
    if(_writer != NULL)
    {
        rmw_qos_profile_t profile = qos.get_rmw_qos_profile();
        rmw_time_t infinity = RMW_DURATION_INFINITE;
        std::string serialized_profiles =
            "- history: "+ std::to_string(profile.history) +"\n"
            "  depth: "+ std::to_string(profile.depth) +"\n"
            "  reliability: "+ std::to_string(profile.reliability) +"\n"
            "  durability: "+ std::to_string(profile.durability) +"\n"
            "  deadline:\n"
            "    sec: " + std::to_string(infinity.sec) + "\n"
            "    nsec: " + std::to_string(infinity.nsec) + "\n"
            "  lifespan:\n"
            "    sec: " + std::to_string(infinity.sec) + "\n"
            "    nsec: " + std::to_string(infinity.nsec) + "\n"
            "  liveliness: "+ std::to_string(profile.liveliness) +"\n"
            "  liveliness_lease_duration:\n"
            "    sec: " + std::to_string(infinity.sec) + "\n"
            "    nsec: " + std::to_string(infinity.nsec) + "\n"
            "  avoid_ros_namespace_conventions: false";

        _writer->create_topic(
            {topic_name,
            msg_type,
            rmw_get_serialization_format(),
            serialized_profiles});
    }
}

}