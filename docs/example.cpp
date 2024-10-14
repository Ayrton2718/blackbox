#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <blackbox/blackbox.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

class TestNode : public blackbox::BlackBoxNode
{
private:
    blackbox::Diagnostic                                                _diag1;     // エラー監視
    blackbox::Logger                                                    _log1_info;      // ロガー
    blackbox::Logger                                                    _log1_debug;      // ロガー
    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true>     _rec1;      // 変数のレコード（Plotterと違い独自メッセージも可）
    blackbox::PubRecord<std_msgs::msg::String, true>                    _pub1;    // Publisher + レコード
    blackbox::SubRecord<std_msgs::msg::String, true>                    _sub1;    // Subscription + レコード

    int _counter = 0;

    CallbackGroup::SharedPtr _cb_grp1;
    TimerBase::SharedPtr     _main_tim;

public:
    TestNode(const std::string &name_space = "") 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::DEBUG, "test_node", name_space)
    {
        _diag1.init(this, "diag1");
        _log1_info.init(this, blackbox::INFO, "log1");
        _log1_debug.init(this, blackbox::DEBUG, "log1");
        _rec1.init(this, "rec1");
        _pub1.init(this, "pub1");
        _sub1.init(this, "pub1", 10, [this](std_msgs::msg::String::SharedPtr msg){
                TAGGER(_log1_debug, msg->data);
            });

        // 時間周期を指定して，timer_callbackを登録
        this->_cb_grp1 = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
        this->_main_tim = this->create_wall_timer(10ms, std::bind(&TestNode::timer_callback, this), this->_cb_grp1);
    }

   // 10msに一回呼ばれる．
    void timer_callback() 
    {
        _diag1.ok("OK");

        TAGGER(_log1_info, "status : OK");

        std_msgs::msg::Float32MultiArray msg;
        msg.data.push_back((float)(_counter % 100) / 100);
        _rec1.record(msg);
        _counter++;

        std_msgs::msg::String str;
        str.data = "test";
        _pub1.publish(str);
    }
};