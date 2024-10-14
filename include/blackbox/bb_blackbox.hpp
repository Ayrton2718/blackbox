#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>
#include <rosbag2_cpp/writer.hpp>

namespace blackbox
{

enum class storage_profile_t {
    fastwrite,  // Configures the MCAP writer for the highest possible write throughput and lowest resource utilization. This preset does not calculate CRCs for integrity checking, and does not write a message index. Useful for resource-constrained robots.
    zstd_fast,  // Configures the MCAP writer to use chunk compression with zstd at the lowest compression ratio and disables CRC calculation, to achieve high throughput while conserving disk space.
    zstd_small  // Configures the MCAP writer to write 4MB chunks, compressed with zstd using its highest compression ratio. Calculates chunk CRCs, useful when using ros2 bag convert as a post-processing step.
};

enum class debug_mode_t{
    RELEASE,
    DEBUG,
};


/// @brief 
class BlackBox
{
public:
    template<typename MessageT>
    class BlackBoxWriter
    {
    public:
        BlackBoxWriter(){}

        void BlackBoxWriter_cons(BlackBox* handle, std::string topic_name, size_t drop_count=0, const rclcpp::QoS& qos=rclcpp::SystemDefaultsQoS())
        {
            _handle = handle;
            _topic_name = topic_name;
            _counter = 0;
            _drop_count = (drop_count + 1);
            _handle->create(topic_name, rosidl_generator_traits::name<MessageT>(), qos);
        }

        void write(MessageT msg){
            this->write(msg, this->get_bb_tim());
        }

        void write(MessageT msg, rclcpp::Time tim)
        {
            if((_handle != NULL) && ((_counter % _drop_count) == 0))
            {
                std::shared_ptr<rclcpp::SerializedMessage> serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                _serialization.serialize_message(&msg, serialized_msg.get());
                _handle->write(serialized_msg, _topic_name, rosidl_generator_traits::name<MessageT>(), tim);
            }
            _counter++;
        }

        void write(std::shared_ptr<MessageT> msg){
            this->write(msg, this->get_bb_tim());
        }

        void write(std::shared_ptr<MessageT> msg, rclcpp::Time tim)
        {
            if((_handle != NULL) && ((_counter % _drop_count) == 0))
            {
                std::shared_ptr<rclcpp::SerializedMessage> serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
                _serialization.serialize_message(msg.get(), serialized_msg.get());
                _handle->write(serialized_msg, _topic_name, rosidl_generator_traits::name<MessageT>(), tim);
            }
            _counter++;
        }

        rclcpp::Time get_bb_tim(void){
            return _handle->get_bb_tim();
        }

    private:
        BlackBox*       _handle = nullptr;
        std::string     _topic_name;

        size_t      _counter;
        size_t      _drop_count;
        rclcpp::Serialization<MessageT> _serialization;
    };

private:
    rclcpp::Clock*   _clock;
    std::unique_ptr<rosbag2_cpp::Writer> _writer = NULL;

    void create(std::string topic_name, std::string msg_type, const rclcpp::QoS& qos=rclcpp::SystemDefaultsQoS());

    void write(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg, std::string topic_name, std::string msg_type, rclcpp::Time tim)
    {
        if(_writer != NULL)
        {
            _writer->write(serialized_msg, topic_name, msg_type, tim);
        }
    }

    std::string storage_profile_toString(storage_profile_t config);

public:
    const debug_mode_t    _bb_debug_mode;

    /// @brief 
    /// @tparam MessageT 
    /// @param node 
    /// @param storage_preset_profile 
    /// @param max_cache_size
    BlackBox(rclcpp::Node* node, debug_mode_t debug_mode, std::string file_name="blackbox", storage_profile_t storage_preset_profile=storage_profile_t::zstd_fast, uint64_t max_cache_size = 1024);

    virtual ~BlackBox() noexcept
    {
    }

    rclcpp::Time get_bb_tim(void){
        return _clock->now();
    }
};


template<typename MessageT>
using BlackBoxWriter = BlackBox::BlackBoxWriter<MessageT>;
}