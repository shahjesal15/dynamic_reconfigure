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
    auto result = future.get();
    if (!result)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve list of parameters.");
        return;
    }
    for (auto &name : result->result.names)
    {
        parameters.push_back(name);
    }
    is_ready = true;
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
    for(auto& parameter : parameters) {
        RCLCPP_INFO_STREAM(node_->get_logger(), " ? " << parameter);
    }
    auto type_request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
    type_request->names = std::vector<std::string>{};
    for(auto& parameter : parameters)
        type_request->names.push_back(parameter);//{"test"};

    auto list_future = describe_parameters_client_->async_send_request(type_request,
                                                                       std::bind(&reconfigure_depends::ServiceWrapper::update_parameter_types_cb, this, std::placeholders::_1));
}

void reconfigure_depends::ServiceWrapper::update_parameter_types_cb(
    const rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture future)
{

    auto result = future.get();
    if (!result)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve list of parameters.");
        return;
    }
    for (auto &descriptor : result->descriptors)
    {
        std::string type_str;
        switch (descriptor.type)
        {
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
            type_str = "bool";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
            type_str = "integer";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
            type_str = "double";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
            type_str = "string";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
            type_str = "byte array";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
            type_str = "bool array";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
            type_str = "integer array";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
            type_str = "double array";
            break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
            type_str = "string array";
            break;
        default:
            type_str = "unknown";
        }
        parameter_type[descriptor.name] = type_str;
        RCLCPP_INFO_STREAM(node_->get_logger(), " > name : " << descriptor.name << ", type : " << type_str);
    }
}

std::vector<std::string> &reconfigure_depends::ServiceWrapper::list_parameters(std::string &node_name)
{
    if (prev_accessed_node != node_name)
    {
        create_client(node_name);
        update_parameters();
        while(parameters.size() == 0);
        update_parameter_types();
        prev_accessed_node = node_name;
    }
    return parameters;
}

reconfigure_depends::ServiceWrapperReturnCodes reconfigure_depends::ServiceWrapper::set_param(const std::string &name, const rclcpp::ParameterValue &value)
{
}

rclcpp::ParameterValue reconfigure_depends::ServiceWrapper::get_param(const std::string &name)
{
}