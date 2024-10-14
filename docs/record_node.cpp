#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <blackbox/blackbox.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class BlackBoxRecordNode : public rclcpp::Node, public blackbox::BlackBox
{
private:
    // blackbox::SubRecord<rcl_interfaces::msg::Log, true> _rosout;
    blackbox::SubRecord<diagnostic_msgs::msg::DiagnosticArray, true> _diagnostics;
    blackbox::SubRecord<tf2_msgs::msg::TFMessage, true> _tf;
    blackbox::SubRecord<tf2_msgs::msg::TFMessage, true> _tf_static;

public:
    BlackBoxRecordNode(const rclcpp::NodeOptions &options) : BlackBoxRecordNode("", options) {}
    BlackBoxRecordNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
        : rclcpp::Node("blackbox_record_node", name_space, options), blackbox::BlackBox(this, blackbox::debug_mode_t::RELEASE)
    {
        // _rosout.init(this, this, "/rosout", rclcpp::RosoutQoS(), [](rcl_interfaces::msg::Log::SharedPtr msg){(void)msg;});
        _diagnostics.init(this, this, "/diagnostics", 1, [](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg){(void)msg;});
        _tf.init(this, this, "/tf", rclcpp::ServicesQoS(), [](tf2_msgs::msg::TFMessage::SharedPtr msg){(void)msg;});
        _tf_static.init(this, this, "/tf_static", rclcpp::RosoutQoS(), [](tf2_msgs::msg::TFMessage::SharedPtr msg){(void)msg;});
    }

    virtual ~BlackBoxRecordNode() noexcept override{}
};