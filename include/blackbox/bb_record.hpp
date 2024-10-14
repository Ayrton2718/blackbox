#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>

#include "bb_blackbox.hpp"
#include "bb_node.hpp"

namespace blackbox
{

template<typename MessageT, bool IS_ENABLE_TOPIC=false, bool IS_ENABLE_RECORD=true>
class Record : private BlackBoxWriter<MessageT>
{
public:
    Record() : BlackBoxWriter<MessageT>(){
    }

    void init(blackbox::BlackBoxNode* node, std::string topic_name, size_t drop_count=0, const rclcpp::QoS& qos=rclcpp::SensorDataQoS()){
        this->init(static_cast<rclcpp::Node*>(node), static_cast<BlackBox*>(node), topic_name, drop_count, qos);
    }

    void init(rclcpp::Node* node, BlackBox* handle, std::string record_name, size_t drop_count=0, const rclcpp::QoS& qos=rclcpp::SensorDataQoS()){
        std::string ns = node->get_namespace();
        if(ns.size() != 1){
            ns += '/';
        }
        
        if(IS_ENABLE_TOPIC)
            _publisher = node->create_publisher<MessageT>("/record" + ns + record_name, qos);

        if(IS_ENABLE_RECORD)
            BlackBoxWriter<MessageT>::BlackBoxWriter_cons(handle, "/record" + ns + record_name, drop_count, qos);
    }

    void record(MessageT msg){
        if(IS_ENABLE_RECORD){
            this->record(msg, this->get_bb_tim());
        }else{
            this->record(msg, rclcpp::Clock().now());
        }
    }

    void record(MessageT msg, rclcpp::Time tim)
    {
        if(IS_ENABLE_TOPIC)
            _publisher->publish(msg);

        if(IS_ENABLE_RECORD)
            BlackBoxWriter<MessageT>::write(msg, tim);
    }

private:
    typename rclcpp::Publisher<MessageT>::SharedPtr _publisher;
};

}
