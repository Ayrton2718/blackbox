#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <blackbox/blackbox.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

// blackbox::BlackBoxNodeを継承する
class TestNode : public blackbox::BlackBoxNode
{
private:
    // Loggerのインスタンスを作成
    blackbox::Logger                                                    _log1_info;     // _log1_info：情報メッセージをログに記録するためのもの。
    blackbox::Logger                                                    _log1_debug;    // _log1_debug：デバッグメッセージをログに記録するためのもの。

    // Recordのインスタンスを作成
    // std_msgs::msg::Float32MultiArrayはメッセージ型, falseはTopicするかを指定する（デフォルトはfalse）, trueはレコードするかを指定する（デフォルトはtrue）
    blackbox::Record<std_msgs::msg::Float32MultiArray, false, true>     _rec1;

    // Publisher + レコードのインスタンスを作成
    // std_msgs::msg::Stringはメッセージ型, trueは記録を有効にするかを指定する（デフォルトはtrue）
    blackbox::PubRecord<std_msgs::msg::String, true>                    _pub1;

    // Subscriber + レコードのインスタンスを作成
    // std_msgs::msg::Stringはメッセージ型, trueは記録を有効にするかを指定する（デフォルトはtrue）
    blackbox::SubRecord<std_msgs::msg::String, false>                    _sub1;

    int _counter = 0;

    CallbackGroup::SharedPtr _cb_grp1;
    TimerBase::SharedPtr     _main_tim;

public:
    // コンストラクタ
    // debug_mode_t::DEBUGはデバッグモードを指定する．主にログをコンソールに出力するかどうかを指定する．
    TestNode(const std::string &name_space = "") 
        : blackbox::BlackBoxNode(blackbox::debug_mode_t::DEBUG, "test_node", name_space)
    {        
        // Loggerの初期化
        // thisはBlackBoxNodeクラスのインスタンスを指す．
        // blackbox::INFOとblackbox::DEBUGはログの重要度を表す．
        // blackbox::INFOは常にレコードされる．blackbox::DEBUGはデバッグモード（blackbox::debug_mode_t::DEBUG）のときだけレコードされる．
        // "log1"はログのタグ名を表す．
        // レコードされるtopic名は"/tagger/namespace/test_node"になる．（レコードされる型はrcl_interfaces::msg::Log）
        _log1_info.init(this, blackbox::INFO, "log1");
        _log1_debug.init(this, blackbox::DEBUG, "log1");

        // Recordの初期化
        // "rec1"はタグ名を表す． 
        // レコードされるtopic名は"/record/namespace/rec1"になる．
        _rec1.init(this, "rec1");

        // Publisher + レコードの初期化
        // "/pub1"はTopic名を表す．"pub1"とするとネームスペースが付与される．
        // レコードされるtopic名は"/pub1"になる．
        _pub1.init(this, "/pub1");

        // Subscriber + レコードの初期化
        // "/pub1"はTopic名を表す．"pub1"とするとネームスペースが付与される．
        // レコードされるtopic名は"/pub1"になる．
        _sub1.init(this, "/pub1", 10, [this](std_msgs::msg::String::SharedPtr msg){
                // ログの記録, _log1_infoはblackbox::Loggerのインスタンス
                TAGGER(_log1_debug, msg->data);
            });

        // 時間周期を指定して，timer_callbackを登録
        this->_cb_grp1 = this->create_callback_group(CallbackGroupType::MutuallyExclusive);
        this->_main_tim = this->create_wall_timer(10ms, std::bind(&TestNode::timer_callback, this), this->_cb_grp1);
    }

   // 10msに一回呼ばれる．
    void timer_callback() 
    {
        // ログの記録, _log1_infoはblackbox::Loggerのインスタンス
        TAGGER(_log1_info, "status : OK");

        // レコードの記録, _rec1はblackbox::Recordのインスタンス
        std_msgs::msg::Float32MultiArray msg;
        msg.data.push_back((float)(_counter % 100) / 100);
        _rec1.record(msg);
        _counter++;

        // レコードされたメッセージをpublishする, _pub1はblackbox::PubRecordのインスタンス
        std_msgs::msg::String str;
        str.data = "test";
        _pub1.publish(str);
    }
};