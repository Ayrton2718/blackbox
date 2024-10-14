#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace blackbox
{

/// @brief 
class DiagnosticUpdater
{
public:
    class Diagnostic
    {
    private:
        void set_diagnostic(unsigned char lvl, std::string msg){
            volatile uint8_t flg = (_flip_flg + 1) % 2;
            _lvl[flg] = lvl;
            _msg[flg] = msg;
            _flip_flg = flg;
        }

    public:
        Diagnostic(){}

        void init(DiagnosticUpdater* parent, std::string name){
            _flip_flg = 0;
            _lvl[0] = 0;
            _lvl[1] = 0;
            _msg[0] = "";
            _msg[1] = "";

            parent->_updater.add(name, [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
                        volatile uint8_t flg = _flip_flg;
                        stat.summary(_lvl[flg], _msg[flg]);
                    });
        }

        void ok(std::string msg=""){
            this->set_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, msg);
        }

        void warn(std::string msg){
            this->set_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN, msg);
        }

        void error(std::string msg){
            this->set_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR, msg);
        }

    private:
        volatile uint8_t _flip_flg;
        unsigned char _lvl[2];
        std::string _msg[2];
    };

private:
    diagnostic_updater::Updater _updater;

public:
    DiagnosticUpdater(rclcpp::Node* node, std::string hardware_id, float period=0.5) : _updater(node, period)
    {
        _updater.setHardwareID(hardware_id);
    }

    virtual ~DiagnosticUpdater(){
    }

    void diagnostic_bind(std::string name, diagnostic_updater::TaskFunction task){
        this->_updater.add(name, task);
    }
};

using Diagnostic = DiagnosticUpdater::Diagnostic;

}
