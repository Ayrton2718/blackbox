#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>

#include "bb_blackbox.hpp"
#include "bb_node.hpp"

namespace blackbox
{

template<typename MessageT, bool IS_ENABLE_RECORD=true>
class PubRecord : private BlackBoxWriter<MessageT>
{
public:
    PubRecord() : BlackBoxWriter<MessageT>(){
    }

    void init(blackbox::BlackBoxNode* node, std::string topic_name, const rclcpp::QoS& qos=10, size_t drop_count=0){
        this->init(static_cast<rclcpp::Node*>(node), static_cast<BlackBox*>(node), topic_name, qos, drop_count);
    }

    void init(rclcpp::Node* node, BlackBox* handle, std::string topic_name, const rclcpp::QoS& qos=10, size_t drop_count=0){
        _publisher = node->create_publisher<MessageT>(topic_name, qos);

        if(IS_ENABLE_RECORD)
        {
            std::string ns;
            if(topic_name[0] == '/')
            {
                ns = "";
            }else{
                ns = node->get_namespace();
                if(ns.size() != 1){
                    ns += '/';
                }
            }
            BlackBoxWriter<MessageT>::BlackBoxWriter_cons(handle, ns + topic_name, drop_count, qos);
        }
    }
    
    void publish(MessageT msg){
        if(IS_ENABLE_RECORD){
            this->publish(msg, this->get_bb_tim());
        }else{
            this->publish(msg, rclcpp::Clock().now());
        }
    }

    void publish(MessageT msg, rclcpp::Time tim)
    {        
        _publisher->publish(msg);


        if(IS_ENABLE_RECORD)
            BlackBoxWriter<MessageT>::write(msg, tim);
    }

private:
    typename rclcpp::Publisher<MessageT>::SharedPtr _publisher;

};

}
