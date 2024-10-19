/**
 * @file         : service_wrapper.hpp
 * @author       : Jesal Shah
 * @date         : 06-10-2024
 * @description  : Service manager is heart of dynamic reconfiguration and enables the node 
 *                 to change or access the parameters of other nodes on the fly.
 */

#ifndef SERVICE_WRAPPER_HPP__
#define SERVICE_WRAPPER_HPP__

#include <iostream>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>

namespace reconfigure_depends
{
    enum ServiceWrapperReturnCodes {
        FAILURE = -1,
        SUCCESS = 0,
    };

    class ServiceWrapper
    {
    private:
        /// @brief this function is the callback for the list_parameters service call
        /// @param future 
        void update_parameters_cb(const rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future);
        
        /// @brief this function is the callback for the describe_parameters service call
        /// @param future 
        void update_parameter_types_cb(const rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture future);
        
    protected:
        bool is_ready = false;
        /// @brief holds the information of the last accessed node
        std::string prev_accessed_node;
        
        /// @brief holds the list of the parameters of the previously accessed node
        std::vector<std::string> parameters;
        
        /// @brief holds the map for type of parameter for the given parameter
        std::map<std::string, std::string> parameter_type;
        
        /// @brief updates the parameters of the particular node
        void update_parameters();

        /// @brief updates the parameter types of the particular node
        void update_parameter_types();
        
        /// @brief create the client to access the services
        void create_client(const std::string &node_name);
        
        /// @brief shared pointer to node
        rclcpp::Node::SharedPtr node_;
        
        /// @brief shared pointer to client that can set parameters
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
        
        /// @brief shared pointer to client that can get parameters
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
        
        /// @brief shared pointer to client that can list parameters
        rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
        
        /// @brief shared pointer to client that can describe parameters
        rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_client_;
        
    public:
        /// @brief Constructor for the class
        ServiceWrapper(rclcpp::Node::SharedPtr node);
        
        /// @brief Returns a vector of available parameters for the given node
        /// @param name
        /// @return std::vector<std::string>&
        std::vector<std::string>& list_parameters(std::string &node_name);
        
        /// @brief This function sets the value of the given parameter
        /// @param name
        /// @param value
        /// @return ServiceWrapperReturnCodes
        ServiceWrapperReturnCodes set_param(const std::string &name, const rclcpp::ParameterValue &value);
        
        /// @brief This function gets the value of the given parameter
        /// @param name
        /// @return rclcpp::ParameterValue
        rclcpp::ParameterValue get_param(const std::string &name);
        
        /// @brief Destructor for the class
        ~ServiceWrapper() {}
    };
};

#endif // SERVICE_WRAPPER_HPP__