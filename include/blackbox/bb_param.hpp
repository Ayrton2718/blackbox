#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "bb_logger.hpp"

namespace blackbox
{

/// @brief 
class ParamUpdater
{
private:
    typedef struct{
        std::mutex  lock;
        rclcpp::ParameterValue value;
    }param_t;

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

            auto default_param = rclcpp::ParameterValue(default_value);

            _param_type = default_param.get_type();
            _param_obj = _handle->declare_param(name, default_param);
        }

        ParamT get(void){
            auto param = _handle->get_param(_param_obj);
            return static_cast<ParamT>(param.get_value<ParamT>());
        }

        void set(ParamT new_value){
            auto param_value = rclcpp::ParameterValue(new_value);
            _handle->set_param(_param_obj, _name, param_value);
        }

    private:
        ParamUpdater*               _handle = nullptr;
        std::string                 _name;
        
        rclcpp::ParameterType       _param_type = rclcpp::ParameterType::PARAMETER_NOT_SET;
        std::shared_ptr<param_t>    _param_obj;
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

        auto result = node->list_parameters({}, 10);
        TAGGER(_info, "==== Parameter list ====");
        for (const auto & param_name : result.names) {
            TAGGER(_info, "Parameter %s", param_name.c_str());
        }
        TAGGER(_info, "========================");

        if(enable_update)
        {
            _param_callback_handle = _node->add_on_set_parameters_callback(
                std::bind(&ParamUpdater::onParameterChange, this, std::placeholders::_1)
            );
        }
    }
    
    std::shared_ptr<param_t> declare_param(std::string name, const rclcpp::ParameterValue& default_param){
        std::shared_ptr<param_t> return_param = nullptr;
        
        {
            auto it = _params.find(name);
            if(it != _params.end())
            {
                return_param = it->second;
                TAGGER(_info, "Parameter %s already exists", name.c_str());
            }
            else
            {
                rclcpp::Parameter new_param;
                _node->declare_parameter(name, default_param);
                new_param = _node->get_parameter(name);
                TAGGER(_info, "Parameter %s declared", name.c_str());

                return_param = std::make_shared<param_t>();
                return_param->value = new_param.get_parameter_value();

                _locker.lock();
                _params[name] = return_param;
                _locker.unlock();
            }
        }

        if(return_param->value.get_type() != default_param.get_type()){
            auto return_str = rclcpp::Parameter(name, return_param->value).value_to_string();
            auto default_str = rclcpp::Parameter(name, default_param).value_to_string();
            TAGGER(_info, "Parameter %s type error, %s -> %s", name.c_str(), default_str.c_str(), return_str.c_str());

            return_param = nullptr;
        }

        return return_param;
    }

    void set_param(std::shared_ptr<param_t> obj, std::string name, const rclcpp::ParameterValue& param_value){
        _node->set_parameter(rclcpp::Parameter(name, param_value));

        std::lock_guard<std::mutex> lock(obj->lock);
        obj->value = param_value;
    }

    rclcpp::Parameter get_param(std::shared_ptr<param_t> obj){
        std::lock_guard<std::mutex> lock(obj->lock);
        return rclcpp::Parameter("", obj->value);
    }

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        {
            std::lock_guard<std::mutex> lock(_locker);
            for (const rclcpp::Parameter& param : parameters) {
                auto it = _params.find(param.get_name());
                if (it != _params.end()) {
                    std::lock_guard<std::mutex> lock(it->second->lock);
                    if(it->second->value.get_type() == param.get_type()){
                        it->second->value = param.get_parameter_value();
                        TAGGER(_info, "Parameter %s updated, value: %s", param.get_name().c_str(), param.value_to_string().c_str());
                    }else{
                        TAGGER(_info, "Parameter %s type error, %s", param.get_name().c_str(), param.value_to_string().c_str());
                    }
                }else{
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
    std::map<std::string, std::shared_ptr<param_t>> _params;
};



template<typename ParamT>
using Param = ParamUpdater::Param<ParamT>;

}
