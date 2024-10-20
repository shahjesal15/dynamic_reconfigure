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
#include <atomic>
#include <mutex>
#include <algorithm>

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
        NOT_FOUND,
        IN_PROCESS,
        OCCUPIED
    };

    enum ServiceWrapperState {
        START = -1,
        READY = 0,
        ERROR,
        BUSY, 
        COMPLETE
    };

    using AtomicServiceWrapperState = std::atomic<ServiceWrapperState>;

    class ServiceWrapper
    {
    private:
        /// @brief this function is the callback for the list_parameters service call
        /// @param future 
        void update_parameters_cb(const rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future);
        
        /// @brief this function is the callback for the describe_parameters service call
        /// @param future 
        void update_parameter_types_cb(const rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture future);

        /// @brief this function is the callback for the set_parameters_atomically service call
        /// @param future 
        void set_param_cb(const rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future);
        
    protected:
        /// @brief holds the information of the last accessed node
        std::string prev_accessed_node;
        
        /// @brief holds the list of the parameters of the previously accessed node
        std::vector<std::string> parameters;
        
        /// @brief holds the map for type of parameter for the given parameter
        std::map<std::string, int> parameter_type;
        
        /// @brief updates the parameters of the particular node
        void update_parameters();

        /// @brief updates the parameter types of the particular node
        void update_parameter_types();
        
        /// @brief create the client to access the services
        void create_client(const std::string &node_name);
        
        /// @brief shared pointer to node
        rclcpp::Node::SharedPtr node_;
        
        /// @brief shared pointer to client that can set parameters
        rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_client_;
        
        /// @brief shared pointer to client that can get parameters
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
        
        /// @brief shared pointer to client that can list parameters
        rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
        
        /// @brief shared pointer to client that can describe parameters
        rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_client_;

        /// @brief state to check if the list updated or not
        AtomicServiceWrapperState is_list_updated;

        /// @brief state to check if the parameter is set or not
        AtomicServiceWrapperState is_param_set;

        /// @brief mutex for service clients
        std::mutex client_mutex;

        /// @brief mutex for parameters read from the list parameters callback
        std::mutex parameters_mutex;
        
        /// @brief mutex for parameters types read from the describe parameters callback
        std::mutex parameter_types_mutex;

    public:
        /// @brief Constructor for the class
        ServiceWrapper(rclcpp::Node::SharedPtr node);
        
        /// @brief Returns a vector of available parameters for the given node
        /// @param name
        /// @return std::vector<std::string>&
        ServiceWrapperReturnCodes list_parameters(std::string &node_name);
        
        /// @brief This function sets the value of the given parameter
        /// @param name
        /// @param value
        /// @return ServiceWrapperReturnCodes
        ServiceWrapperReturnCodes set_param(const std::string &name, const rclcpp::ParameterValue &value);
        
        /// @brief This function gets the value of the given parameter
        /// @param name
        /// @return rclcpp::ParameterValue
        rclcpp::ParameterValue get_param(const std::string &name);
        
        /// @brief this function returns the state of the list update service callback
        /// @return ServiceWrapperState
        ServiceWrapperState get_list_status();

        /// @brief this function returns the parameters that were received on listing the parameters 
        /// @return 
        std::vector<std::string> get_parameters();

        /// @brief Destructor for the class
        ~ServiceWrapper() {}
    };
};

#endif // SERVICE_WRAPPER_HPP__