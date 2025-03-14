#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>

#include "bb_blackbox.hpp"
#include "bb_node.hpp"

namespace blackbox
{

/// @brief Subscriber + レコードのクラス
/// @tparam MessageT メッセージ型
/// @tparam IS_ENABLE_RECORD レコードを有効にするかを指定する．（デフォルトはtrue）
template<typename MessageT, bool IS_ENABLE_RECORD=true>
class SubRecord : private BlackBoxWriter<MessageT>
{
public:
    SubRecord() : BlackBoxWriter<MessageT>(){
    }

    /// @brief 初期化
    /// @param node blackbox::BlackBoxNodeのポインタ
    /// @param topic_name Topic名（"/"から始まる場合はそのまま、そうでない場合はノードのネームスペースを付与）
    /// @param qos QoS
    /// @param callback コールバック関数
    /// @param drop_count ドロップ数（0はドロップなし）
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

    /// @brief 初期化
    /// @param node rclcpp::Nodeのポインタ
    /// @param handle blackbox::BlackBoxのポインタ
    /// @param topic_name Topic名（"/"から始まる場合はそのまま、そうでない場合はノードのネームスペースを付与）
    /// @param qos QoS
    /// @param callback コールバック関数
    /// @param drop_count ドロップ数（0はドロップなし）
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
        std::string resolve_name = node->get_node_topics_interface()->resolve_topic_name(topic_name);

        if(IS_ENABLE_RECORD)
        {
            std::string ns;
            if(resolve_name[0] == '/')
            {
                ns = "";
            }else{
                ns = node->get_namespace();
                if(ns.size() != 1){
                    ns += '/';
                }
            }
            BlackBoxWriter<MessageT>::BlackBoxWriter_cons(handle, ns + resolve_name, drop_count, qos);
            printf("%s, %s\n", ns.c_str(), resolve_name.c_str());
            fflush(stdout);
        }

        auto cur_callback = [this, callback = std::forward<CallbackT>(callback)](typename std::shared_ptr<MessageT> msg){
                callback(msg);

                if(IS_ENABLE_RECORD)
                    BlackBoxWriter<MessageT>::write(msg);
            };

        _subscription = node->create_subscription<MessageT>(resolve_name, qos, cur_callback, options, msg_mem_strat);
    }

private:
    typename rclcpp::Subscription<MessageT>::SharedPtr _subscription;

};

}
