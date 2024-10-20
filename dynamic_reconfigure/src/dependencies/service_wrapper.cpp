#include "dependencies/service_wrapper.hpp"

reconfigure_depends::ServiceWrapper::ServiceWrapper(rclcpp::Node::SharedPtr node) : node_(node)
{
    prev_accessed_node = "";
    parameters.clear();
    parameter_type.clear();
    is_list_updated.store(ServiceWrapperState::START);
    is_param_set.store(ServiceWrapperState::START);
}

void reconfigure_depends::ServiceWrapper::create_client(const std::string &node_name)
{
    std::lock_guard client_lock(client_mutex);

    set_parameters_client_ = node_->create_client<rcl_interfaces::srv::SetParametersAtomically>("/" + node_name + "/set_parameters_atomically");
    get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/" + node_name + "/get_parameters");
    list_parameters_client_ = node_->create_client<rcl_interfaces::srv::ListParameters>("/" + node_name + "/list_parameters");
    describe_parameters_client_ = node_->create_client<rcl_interfaces::srv::DescribeParameters>("/" + node_name + "/describe_parameters");
}

void reconfigure_depends::ServiceWrapper::update_parameters_cb(
    const rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future)
{   
    auto result = future.get();
    if (!result)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve list of parameters.");
        is_list_updated.store(ServiceWrapperState::ERROR);
        return;
    }

    parameters_mutex.lock();
    for (auto &name : result->result.names)
    {
        parameters.push_back(name);
        RCLCPP_DEBUG_STREAM(node_->get_logger(), " [@] Parameter : " << name);
    }
    parameters_mutex.unlock();
    
    update_parameter_types();
}

void reconfigure_depends::ServiceWrapper::update_parameters()
{
    std::lock_guard client_lock(client_mutex);

    auto list_request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    list_request->depth = rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE;

    auto list_future = list_parameters_client_->async_send_request(list_request,
                                                                   std::bind(&reconfigure_depends::ServiceWrapper::update_parameters_cb, this, std::placeholders::_1));
}

void reconfigure_depends::ServiceWrapper::update_parameter_types_cb(
    const rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture future)
{
    auto result = future.get();
    if (!result)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve list of parameters.");
        is_list_updated.store(ServiceWrapperState::ERROR);
        return;
    }

    parameter_types_mutex.lock();
    for (auto &descriptor : result->descriptors)
    {
        parameter_type[descriptor.name] = descriptor.type;
        RCLCPP_DEBUG_STREAM(node_->get_logger(), 
            " [@] Parameter : " << descriptor.name << ", type : " << parameter_type[descriptor.name]);
    }
    parameter_types_mutex.unlock();

    is_list_updated.store(ServiceWrapperState::READY);
}

void reconfigure_depends::ServiceWrapper::update_parameter_types()
{
    std::lock_guard client_lock(client_mutex);

    auto type_request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();

    parameters_mutex.lock();
    for(std::string& parameter : parameters)
        type_request->names.push_back(parameter);
    parameters_mutex.unlock();

    auto list_future = describe_parameters_client_->async_send_request(type_request,
                                                                       std::bind(&reconfigure_depends::ServiceWrapper::update_parameter_types_cb, this, std::placeholders::_1));
}

reconfigure_depends::ServiceWrapperReturnCodes reconfigure_depends::ServiceWrapper::list_parameters(
    std::string &node_name)
{
    if (prev_accessed_node != node_name || is_list_updated.load() == ServiceWrapperState::ERROR)
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), " [+] Getting parameters for " << node_name);

        is_list_updated.store(ServiceWrapperState::BUSY);

        create_client(node_name);
        update_parameters();
        prev_accessed_node = node_name;
    }
    return ServiceWrapperReturnCodes::SUCCESS;
}

reconfigure_depends::ServiceWrapperState reconfigure_depends::ServiceWrapper::get_list_status() {

    if(is_list_updated.load() == ServiceWrapperState::READY) {
        is_list_updated.store(ServiceWrapperState::COMPLETE);
        return ServiceWrapperState::READY;
    }
    return is_list_updated.load();
}

std::vector<std::string> reconfigure_depends::ServiceWrapper::get_parameters() {
    std::lock_guard parameters_lock(parameters_mutex);
    return parameters;
}

void reconfigure_depends::ServiceWrapper::set_param_cb(const rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future) {
    auto result = future.get();
    if(!result) {
        is_param_set.store(ServiceWrapperState::ERROR);
        RCLCPP_WARN(node_->get_logger(), " [-] parameter set failure, with unknown error");
        return;
    }
    if(result->result.successful == true) {
        is_param_set.store(ServiceWrapperState::COMPLETE);
        RCLCPP_DEBUG(node_->get_logger(), " [+] parameter set successfully.");
    } else {
        is_param_set.store(ServiceWrapperState::ERROR);
        RCLCPP_WARN_STREAM(node_->get_logger(), " [-] parameter set failure : " << result->result.reason);
    }
}

reconfigure_depends::ServiceWrapperReturnCodes reconfigure_depends::ServiceWrapper::set_param(const std::string &name, const rclcpp::ParameterValue &value)
{

    if(!(is_param_set.load() == ServiceWrapperState::COMPLETE || is_param_set.load() == ServiceWrapperState::START))
        return ServiceWrapperReturnCodes::OCCUPIED;

    if(!(is_list_updated.load() == ServiceWrapperState::READY || is_list_updated.load() == ServiceWrapperState::COMPLETE))
        return ServiceWrapperReturnCodes::IN_PROCESS;

    std::lock_guard set_lock(client_mutex);

    parameters_mutex.lock();
    if(std::find(parameters.begin(), parameters.end(), name) == parameters.end()) {
        RCLCPP_WARN_STREAM(node_->get_logger(), " [!] Trying to set no-existent parameter " << name);
        parameters_mutex.unlock();
        return ServiceWrapperReturnCodes::NOT_FOUND;
    } else {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), " [@] Parameter found and trying to set : " << name);
    }
    parameters_mutex.unlock();

    rclcpp::Parameter to_set_parameter(name, value);

    auto set_param_request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    set_param_request->parameters.push_back(to_set_parameter.to_parameter_msg());

    is_param_set.store(ServiceWrapperState::BUSY);

    auto set_param_future = set_parameters_client_->async_send_request(
        set_param_request,
        std::bind(&reconfigure_depends::ServiceWrapper::set_param_cb, this, std::placeholders::_1)
    ); 

    return ServiceWrapperReturnCodes::SUCCESS;
}

rclcpp::ParameterValue reconfigure_depends::ServiceWrapper::get_param(const std::string &name)
{
}