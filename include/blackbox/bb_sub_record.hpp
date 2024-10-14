#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>

#include "bb_blackbox.hpp"
#include "bb_node.hpp"

namespace blackbox
{

template<typename MessageT, bool IS_ENABLE_RECORD=true>
class SubRecord : private BlackBoxWriter<MessageT>
{
public:
    SubRecord() : BlackBoxWriter<MessageT>(){
    }

    template<
        typename CallbackT,
        typename AllocatorT = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
        typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType
    >   
    void init(
        blackbox::BlackBoxNode* node,
        const std::string & topic_name,
        const rclcpp::QoS & qos,
        CallbackT && callback,
        size_t drop_count=0,
        const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
        rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
        typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
        MessageMemoryStrategyT::create_default()))
    {
        this->init(static_cast<rclcpp::Node*>(node), static_cast<BlackBox*>(node), topic_name, qos, callback, drop_count, options, msg_mem_strat);
    }

    template<
        typename CallbackT,
        typename AllocatorT = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
        typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType
    >    
    void init(
        rclcpp::Node* node, 
        BlackBox* handle,
        const std::string & topic_name,
        const rclcpp::QoS & qos,
        CallbackT && callback,
        size_t drop_count=0,
        const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
        rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
        typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (
        MessageMemoryStrategyT::create_default()))
    {
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
            printf("%s, %s\n", ns.c_str(), topic_name.c_str());
            fflush(stdout);
        }

        auto cur_callback = [this, callback = std::forward<CallbackT>(callback)](typename std::shared_ptr<MessageT> msg){
                callback(msg);

                if(IS_ENABLE_RECORD)
                    BlackBoxWriter<MessageT>::write(msg);
            };

        _subscription = node->create_subscription<MessageT>(topic_name, qos, cur_callback, options, msg_mem_strat);
    }

private:
    typename rclcpp::Subscription<MessageT>::SharedPtr _subscription;

};

}
