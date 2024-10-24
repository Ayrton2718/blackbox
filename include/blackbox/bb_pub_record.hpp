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
/// @tparam IS_ENABLE_RECORD レコードを有効にするかを指定する．（デフォルトはtrue）
template<typename MessageT, bool IS_ENABLE_RECORD=true>
class PubRecord : private BlackBoxWriter<MessageT>
{
public:
    PubRecord() : BlackBoxWriter<MessageT>(){
    }

    /// @brief 初期化
    /// @param node blackbox::BlackBoxNodeのポインタ
    /// @param topic_name Topic名（"/"から始まる場合はそのまま、そうでない場合はノードのネームスペースを付与）
    /// @param qos QoS
    /// @param drop_count ドロップ数（0はドロップなし）
    void init(blackbox::BlackBoxNode* node, std::string topic_name, const rclcpp::QoS& qos=10, size_t drop_count=0){
        this->init(static_cast<rclcpp::Node*>(node), static_cast<BlackBox*>(node), topic_name, qos, drop_count);
    }

    /// @brief 初期化
    /// @param node rclcpp::Nodeのポインタ
    /// @param handle blackbox::BlackBoxのポインタ
    /// @param topic_name Topic名（"/"から始まる場合はそのまま、そうでない場合はノードのネームスペースを付与）
    /// @param qos QoS
    /// @param drop_count ドロップ数（0はドロップなし）
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
    
    /// @brief メッセージの送信
    /// @param msg メッセージ
    void publish(MessageT msg){
        if(IS_ENABLE_RECORD){
            this->publish(msg, this->get_bb_tim());
        }else{
            this->publish(msg, rclcpp::Clock().now());
        }
    }

    /// @brief メッセージの送信
    /// @param msg メッセージ
    /// @param tim レコード用のタイムスタンプ
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
