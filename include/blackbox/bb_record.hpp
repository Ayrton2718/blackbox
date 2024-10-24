#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sys/time.h>

#include "bb_blackbox.hpp"
#include "bb_node.hpp"

namespace blackbox
{

/// @brief Publisher + レコードのクラス
/// @tparam MessageT メッセージ型
/// @tparam IS_ENABLE_TOPIC Topicを有効にするかを指定する．（デフォルトはfalse）
/// @tparam IS_ENABLE_RECORD レコードを有効にするかを指定する．（デフォルトはtrue）
template<typename MessageT, bool IS_ENABLE_TOPIC=false, bool IS_ENABLE_RECORD=true>
class Record : private BlackBoxWriter<MessageT>
{
public:
    Record() : BlackBoxWriter<MessageT>(){
    }

    /// @brief 初期化
    /// @param node blackbox::BlackBoxNodeのポインタ
    /// @param record_name レコード名（"/record/namespace/record_name"になる）
    /// @param qos QoS
    /// @param drop_count ドロップ数（0はドロップなし）
    void init(blackbox::BlackBoxNode* node, std::string record_name, size_t drop_count=0, const rclcpp::QoS& qos=rclcpp::SensorDataQoS()){
        this->init(static_cast<rclcpp::Node*>(node), static_cast<BlackBox*>(node), record_name, drop_count, qos);
    }

    /// @brief 初期化
    /// @param node rclcpp::Nodeのポインタ
    /// @param handle blackbox::BlackBoxのポインタ
    /// @param record_name レコード名（"/record/namespace/record_name"になる）
    /// @param qos QoS
    /// @param drop_count ドロップ数（0はドロップなし）
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

    /// @brief メッセージの送信
    /// @param msg メッセージ
    void record(MessageT msg){
        if(IS_ENABLE_RECORD){
            this->record(msg, this->get_bb_tim());
        }else{
            this->record(msg, rclcpp::Clock().now());
        }
    }

    /// @brief メッセージの送信
    /// @param msg メッセージ
    /// @param tim レコード用のタイムスタンプ
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
