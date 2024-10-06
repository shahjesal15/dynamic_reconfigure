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

void reconfigure_depends::ServiceWrapper::update_parameters()
{
    // clear the parameters and parameter types of the previously accessed nodes
    parameters.clear();
    parameter_type.clear();

    auto list_request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    list_request->depth = rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE;

    auto list_future = list_parameters_client_->async_send_request(list_request);

    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), list_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto list_result = list_future.get();

        auto describe_request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
        describe_request->names = list_result->result.names;

        auto describe_future = describe_parameters_client_->async_send_request(describe_request);
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), describe_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto describe_result = describe_future.get();

            RCLCPP_INFO(node_->get_logger(), "List of parameters with types:");
            for (size_t i = 0; i < list_result->result.names.size(); ++i)
            {
                const auto &name = list_result->result.names[i];
                const auto &descriptor = describe_result->descriptors[i];
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
                RCLCPP_INFO(node_->get_logger(), "- %s: %s", name.c_str(), type_str.c_str());
                parameter_type[name] = type_str;
                parameters.push_back(name);
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call describe_parameters service");
        }
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call list_parameters service");
    }
}

std::vector<std::string> &reconfigure_depends::ServiceWrapper::list_parameters(std::string &node_name)
{
    if (prev_accessed_node != node_name)
    {
        create_client(node_name);
        update_parameters();
        prev_accessed_node = node_name;
    }
    return parameters;
}

bool reconfigure_depends::ServiceWrapper::set_param(const std::string &name, const rclcpp::ParameterValue &value)
{
}

rclcpp::ParameterValue reconfigure_depends::ServiceWrapper::get_param(const std::string &name)
{
}