#include "dependencies/service_wrapper.hpp"

namespace dynamic_reconfigure_core
{
    ServiceWrapper::ServiceWrapper(rclcpp::Node::SharedPtr node) : node_(node)
    {
        list_params_status.store(ServiceWrapperStates::IDLE);
        request_params_status.store(ServiceWrapperStates::IDLE);
        set_params_status.store(ServiceWrapperStates::IDLE);
        params.clear();
        parameter_types.clear();
    }

    ServiceWrapperReturnCodes ServiceWrapper::request_params_list(std::string node_name)
    {
        if(list_params_status.load() == ServiceWrapperStates::ERROR)
            list_params_status.store(ServiceWrapperStates::IDLE);
        
        if (list_params_status.load() != ServiceWrapperStates::IDLE)
            return ServiceWrapperReturnCodes::BUSY;

        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            fmt::format(fg(fmt::color::blue), "requesting list parameters from {}", node_name));

        create_client(node_name);

        list_params_status.store(ServiceWrapperStates::PROCESSING);

        auto list_request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
        list_request->depth = rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE;

        auto list_future = list_params_client_->async_send_request(list_request,
                                                                   std::bind(&ServiceWrapper::list_params_cb, this, std::placeholders::_1));

        list_wd_timer = std::make_unique<dynamic_reconfigure_dependencies::WatchDogTimer>("list_wd_timer", 100, [this]() {
            list_params_status.store(ServiceWrapperStates::ERROR);
            list_params_client_.reset();
            list_params_client_ = node_->create_client<rcl_interfaces::srv::ListParameters>("/" + this->node_name + "/list_parameters");
            RCLCPP_WARN_STREAM(node_->get_logger(), this->node_name + " couldn't access params.");
        });

        return ServiceWrapperReturnCodes::SUCCESS;
    }

    ServiceWrapperReturnCodes ServiceWrapper::request_params(const std::vector<std::string> &param_names)
    {
        if (request_params_status.load() != ServiceWrapperStates::IDLE)
            return ServiceWrapperReturnCodes::BUSY;

        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            fmt::format(fg(fmt::color::blue), "requesting parameter values"));

        requested_params_mutex.lock();
        requested_params.clear();
        retrieved_params.clear();

        auto get_param_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

        rcl_interfaces::msg::ParameterValue temp_value;

        for (std::string name : param_names)
        {
            if (std::find(params.begin(), params.end(), name) == params.end())
            {
                RCLCPP_WARN_STREAM(node_->get_logger(),
                                   fmt::format(fg(fmt::color::yellow), "Trying to get no-existent parameter, skipping {}", name));
                continue;
            }
            else
            {
                RCLCPP_DEBUG_STREAM(node_->get_logger(),
                                    fmt::format(fg(fmt::color::blue), " [@] Parameter found and trying to access {}", name));
            }
            retrieved_params[name] = temp_value;
            requested_params.push_back(name);
            get_param_request->names.push_back(name);
        }

        requested_params_mutex.unlock();

        std::lock_guard client_lock(client_mutex);

        request_params_status.store(ServiceWrapperStates::PROCESSING);

        auto get_param_future = get_params_client_->async_send_request(
            get_param_request,
            std::bind(&ServiceWrapper::request_params_cb, this, std::placeholders::_1));

        return ServiceWrapperReturnCodes::SUCCESS;
    }

    ServiceWrapperReturnCodes ServiceWrapper::set_params(const std::vector<rclcpp::Parameter> &parameters)
    {
        if (set_params_status.load() != ServiceWrapperStates::IDLE)
            return ServiceWrapperReturnCodes::BUSY;

        params_mutex.lock();

        auto set_param_request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

        for (auto &param : parameters)
        {
            if (std::find(params.begin(), params.end(), param.get_name()) == params.end())
            {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Trying to set no-existent parameter" << param.get_name());
                continue;
            }
            else
            {
                RCLCPP_DEBUG_STREAM(node_->get_logger(), fmt::format(
                                                             fg(fmt::color::blue), "Parameter found and trying to set : {} ", param.get_name()));
            }

            set_param_request->parameters.push_back(param.to_parameter_msg());
        }

        params_mutex.unlock();

        std::lock_guard set_lock(client_mutex);

        set_params_status.store(ServiceWrapperStates::PROCESSING);

        auto set_param_future = set_params_client_->async_send_request(
            set_param_request,
            std::bind(&ServiceWrapper::set_params_cb, this, std::placeholders::_1));

        return ServiceWrapperReturnCodes::SUCCESS;
    }

    void ServiceWrapper::list_parameter_types()
    {
        std::lock_guard client_lock(client_mutex);

        auto type_request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();

        params_mutex.lock();
        for (std::string &param : params)
            type_request->names.push_back(param);
        params_mutex.unlock();

        auto list_future = describe_params_client_->async_send_request(type_request,
                                                                       std::bind(&ServiceWrapper::list_param_types_cb, this, std::placeholders::_1));
    }

    std::vector<std::string> ServiceWrapper::get_params_list()
    {
        std::lock_guard params_lock(params_mutex);

        if (list_params_status.load() != ServiceWrapperStates::ERROR)
            list_params_status.store(ServiceWrapperStates::IDLE);

        return params;
    }

    std::map<std::string, rcl_interfaces::msg::ParameterValue> ServiceWrapper::get_params()
    {
        std::lock_guard requested_params_lock(requested_params_mutex);

        if (request_params_status.load() != ServiceWrapperStates::ERROR)
            request_params_status.store(ServiceWrapperStates::IDLE);

        return retrieved_params;
    }

    ServiceWrapperStates ServiceWrapper::get_list_status()
    {
        return list_params_status.load();
    }

    ServiceWrapperStates ServiceWrapper::get_request_status()
    {
        return request_params_status.load();
    }

    ServiceWrapperStates ServiceWrapper::get_set_status() {
        if(set_params_status.load() == ServiceWrapperStates::COMPLETE) {
            set_params_status.store(ServiceWrapperStates::IDLE);
            return ServiceWrapperStates::COMPLETE;
        }
        return set_params_status.load();
    }

    void ServiceWrapper::create_client(const std::string node_name)
    {
        std::lock_guard client_lock(client_mutex);

        this->node_name = node_name;

        set_params_client_ = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>("/" + node_name + "/set_parameters_atomically");
        get_params_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/" + node_name + "/get_parameters");
        list_params_client_ = node_->create_client<rcl_interfaces::srv::ListParameters>("/" + node_name + "/list_parameters");
        describe_params_client_ = node_->create_client<rcl_interfaces::srv::DescribeParameters>("/" + node_name + "/describe_parameters");
    }

    void ServiceWrapper::clear_errors()
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(),
                            fmt::format(fg(fmt::color::turquoise), "clearing errors"));
        if (list_params_status.load() == ServiceWrapperStates::ERROR)
            list_params_status.store(ServiceWrapperStates::IDLE);
    }

    void ServiceWrapper::list_params_cb(
        const rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future)
    {
        auto result = future.get();
        if (!result)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                fmt::format(fg(fmt::color::red), "Failed to retrieve list of parameters."));
            list_params_status.store(ServiceWrapperStates::ERROR);
            return;
        }

        list_wd_timer->stop();
        list_wd_timer.reset();

        params_mutex.lock();
        params.clear();

        for (auto &name : result->result.names)
        {
            RCLCPP_DEBUG_STREAM(node_->get_logger(),
                                fmt::format(fg(fmt::color::blue), " [@] Parameter : {}", name));
            params.push_back(name);
        }

        params_mutex.unlock();

        list_parameter_types();
    }

    void ServiceWrapper::list_param_types_cb(
        const rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture future)
    {
        auto result = future.get();
        if (!result)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                fmt::format(fg(fmt::color::blue), "Failed to retrieve list of parameters."));
            list_params_status.store(ServiceWrapperStates::ERROR);
            return;
        }

        params_mutex.lock();
        parameter_types.clear();

        for (auto &descriptor : result->descriptors)
        {
            parameter_types[descriptor.name] = descriptor.type;
            RCLCPP_DEBUG_STREAM(node_->get_logger(), fmt::format(fg(fmt::color::blue),
                                                                 " [@] Parameter : {}\tType : {}", descriptor.name, parameter_types[descriptor.name]));
        }
        params_mutex.unlock();

        list_params_status.store(ServiceWrapperStates::COMPLETE);
    }

    void ServiceWrapper::request_params_cb(const rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future)
    {
        auto result = future.get();

        uint32_t not_set_params = 0;

        if (!result)
        {
            request_params_status.store(ServiceWrapperStates::ERROR);
            RCLCPP_WARN_STREAM(node_->get_logger(),"parameter get failure, with unknown error");
            return;
        }

        requested_params_mutex.lock();

        if (result->values.size() != requested_params.size())
        {
            request_params_status.store(ServiceWrapperStates::ERROR);
            RCLCPP_WARN_STREAM(node_->get_logger(), "parameter get failure, didn't receive the number of requested parameters");
            return;
        }

        for (uint8_t index = 0; index < result->values.size(); index++)
        {
            if (result->values[index].type == rclcpp::ParameterType::PARAMETER_NOT_SET)
                not_set_params++;
            retrieved_params[requested_params[index]] = result->values[index];
        }

        requested_params_mutex.unlock();

        if (not_set_params > 0)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(),
                               fmt::format(fg(fmt::color::blue), "some of the parameters are not set yet."));
        }

        request_params_status.store(ServiceWrapperStates::COMPLETE);
    }

    void ServiceWrapper::set_params_cb(const rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future)
    {
        auto result = future.get();
        if (!result)
        {
            set_params_status.store(ServiceWrapperStates::ERROR);
            RCLCPP_WARN(node_->get_logger(), "parameter set failure, with unknown error");
            return;
        }
        if (result->result.successful == true)
        {
            set_params_status.store(ServiceWrapperStates::COMPLETE);
            RCLCPP_DEBUG_STREAM(node_->get_logger(), fmt::format(fg(fmt::color::blue), "parameter set successfully."));
        }
        else
        {
            set_params_status.store(ServiceWrapperStates::ERROR);
            RCLCPP_WARN_STREAM(node_->get_logger(), "parameter set failure : " << result->result.reason);
        }
    }
};