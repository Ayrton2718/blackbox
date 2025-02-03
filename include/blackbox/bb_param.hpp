#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "bb_logger.hpp"

namespace blackbox
{

/// @brief 
class ParamUpdater
{
public:
    template<typename ParamT>
    class Param
    {
    public:
        Param(){}

        void init(std::shared_ptr<ParamUpdater> handle, std::string name, ParamT default_value){
            this->init(handle.get(), name, default_value);
        }

        void init(ParamUpdater* handle, std::string name, ParamT default_value){
            _handle = handle;
            _name = name;
            _value = default_value;

            auto param_value = rclcpp::ParameterValue(default_value);
            _param_type = param_value.get_type();

            handle->bind_param(name, param_value, std::bind(&Param<ParamT>::update_param, this, std::placeholders::_1));
        }

        ParamT get(void){
            std::lock_guard<std::mutex> lock(_locker);
            return _value;
        }

        void set(ParamT new_value){
            {
                std::lock_guard<std::mutex> lock(_locker);
                _value = new_value;
            }

            auto param_value = rclcpp::ParameterValue(new_value);
            _handle->set_param(_name, param_value);
        }

    private:
        bool update_param(const rclcpp::Parameter& param){
            if(_param_type == param.get_type()){
                std::lock_guard<std::mutex> lock(_locker);
                _value = static_cast<ParamT>(param.get_value<ParamT>());
                return true;
            }
            return false;
        }

    private:
        ParamUpdater*           _handle = nullptr;
        std::string             _name;
        rclcpp::ParameterType   _param_type = rclcpp::ParameterType::PARAMETER_NOT_SET;

        std::mutex  _locker;
        ParamT      _value;
    };

public:
    template<typename T>
    ParamUpdater(T* node, bool enable_update=false){
        this->ParamUpdater_cons(static_cast<rclcpp::Node*>(node), static_cast<LogRecorder*>(node), enable_update);
    }

private:
    void ParamUpdater_cons(rclcpp::Node* node, LogRecorder* lr, bool enable_update=false){
        _node = node;
        _info.init(lr, blackbox::LIB_INFO, "bb_param");

        if(enable_update)
        {
            _param_callback_handle = _node->add_on_set_parameters_callback(
                std::bind(&ParamUpdater::onParameterChange, this, std::placeholders::_1)
            );
        }
    }
    
    void bind_param(std::string name, const rclcpp::ParameterValue& param_value, std::function<bool(const rclcpp::Parameter&)> function){
        {
            std::lock_guard<std::mutex> lock(_locker);
            _params[name] = function;
        }

        _node->declare_parameter(name, param_value);
    }

    void set_param(std::string name, const rclcpp::ParameterValue& param_value){
        _node->set_parameter(rclcpp::Parameter(name, param_value));
    }

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        {
            std::lock_guard<std::mutex> lock(_locker);
            for (const rclcpp::Parameter& param : parameters) {
                auto it = _params.find(param.get_name());
                if (it != _params.end()) {
                    if(it->second(param)){
                        TAGGER(_info, "Parameter %s updated, value: %s", param.get_name().c_str(), param.value_to_string().c_str());
                    }else{
                        // TODO type error
                        TAGGER(_info, "Parameter %s type error", param.get_name().c_str());
                    }
                }else{
                    // TODO undefined error
                    TAGGER(_info, "Parameter %s undefined", param.get_name().c_str());
                }
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

private:
    rclcpp::Node* _node = nullptr;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _param_callback_handle;

    Logger _info;

    std::mutex _locker;
    std::map<std::string, std::function< bool(const rclcpp::Parameter&)>> _params;
};



template<typename ParamT>
using Param = ParamUpdater::Param<ParamT>;

}
