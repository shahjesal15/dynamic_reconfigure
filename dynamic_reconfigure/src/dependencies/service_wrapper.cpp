#include "dependencies/service_wrapper.hpp"

reconfigure_depends::ServiceWrapper::ServiceWrapper(rclcpp::Node::SharedPtr node) : node_(node)
{
    prev_accessed_node = "";
    parameters.clear();
    parameter_type.clear();
}

void reconfigure_depends::ServiceWrapper::create_client(const std::string &node_name)
{
    set_parameters_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("/" + node_name + "/set_parameters");
    get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/" + node_name + "/get_parameters");
    list_parameters_client_ = node_->create_client<rcl_interfaces::srv::ListParameters>("/" + node_name + "/list_parameters");
    describe_parameters_client_ = node_->create_client<rcl_interfaces::srv::DescribeParameters>("/" + node_name + "/describe_parameters");
}

void reconfigure_depends::ServiceWrapper::update_parameters_cb(
    const rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future)
{
    list_mutex.lock();
       
    auto result = future.get();
    if (!result)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve list of parameters.");
        is_list_updated = ServiceWrapperState::ERROR;
        return;
    }
    for (auto &name : result->result.names)
    {
        parameters.push_back(name);
        RCLCPP_DEBUG_STREAM(node_->get_logger(), " [@] Parameter : " << name);
    }
    
    list_mutex.unlock();

    update_parameter_types();
}

void reconfigure_depends::ServiceWrapper::update_parameters()
{
    auto list_request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    list_request->depth = rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE;

    auto list_future = list_parameters_client_->async_send_request(list_request,
                                                                   std::bind(&reconfigure_depends::ServiceWrapper::update_parameters_cb, this, std::placeholders::_1));
}

void reconfigure_depends::ServiceWrapper::update_parameter_types()
{
    list_mutex.lock();

    auto type_request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
    for(std::string& parameter : parameters)
        type_request->names.push_back(parameter);

    auto list_future = describe_parameters_client_->async_send_request(type_request,
                                                                       std::bind(&reconfigure_depends::ServiceWrapper::update_parameter_types_cb, this, std::placeholders::_1));


    list_mutex.unlock();
}

void reconfigure_depends::ServiceWrapper::update_parameter_types_cb(
    const rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture future)
{
    auto result = future.get();
    if (!result)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve list of parameters.");
        is_list_updated = ServiceWrapperState::ERROR;
        return;
    }
    for (auto &descriptor : result->descriptors)
    {
        parameter_type[descriptor.name] = descriptor.type;
        RCLCPP_DEBUG_STREAM(node_->get_logger(), 
            " [@] Parameter : " << descriptor.name << ", type : " << parameter_type[descriptor.name]);
    }
    is_list_updated = ServiceWrapperState::READY;
}

reconfigure_depends::ServiceWrapperReturnCodes reconfigure_depends::ServiceWrapper::list_parameters(
    std::string &node_name)
{
    if (prev_accessed_node != node_name)
    {
        list_mutex.lock();

        RCLCPP_DEBUG_STREAM(node_->get_logger(), " [+] Getting parameters for " << node_name);

        is_list_updated = ServiceWrapperState::BUSY;

        parameters.clear();
        create_client(node_name);
        update_parameters();
        prev_accessed_node = node_name;


        list_mutex.unlock();
    }
    return ServiceWrapperReturnCodes::SUCCESS;
}

reconfigure_depends::ServiceWrapperState reconfigure_depends::ServiceWrapper::get_list_status() {
    std::lock_guard<std::mutex> lock(list_mutex);

    if(is_list_updated == ServiceWrapperState::READY) {
        is_list_updated = ServiceWrapperState::COMPLETE;
        return ServiceWrapperState::READY;
    }

    return is_list_updated;
}

std::vector<std::string> reconfigure_depends::ServiceWrapper::get_parameters() {
    std::vector<std::string> parameters_;
    
    list_mutex.lock();
    if(is_list_updated == ServiceWrapperState::COMPLETE)
        parameters_ = parameters;
    list_mutex.unlock();
    
    return parameters;
}


reconfigure_depends::ServiceWrapperReturnCodes reconfigure_depends::ServiceWrapper::set_param(const std::string &name, const rclcpp::ParameterValue &value)
{
}

rclcpp::ParameterValue reconfigure_depends::ServiceWrapper::get_param(const std::string &name)
{
}