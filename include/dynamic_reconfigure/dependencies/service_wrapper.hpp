/**
 * @file         : service_wrapper.hpp
 * @author       : Jesal Shah
 * @date         : 19-03-24
 * @brief        : The Service Manager serves as the core of dynamic reconfiguration,
 *                 allowing nodes to modify or access the parameters of other nodes in real time.
 */

#ifndef SERVICE_WRAPPER_HPP__
#define SERVICE_WRAPPER_HPP__

#include <iostream>
#include <vector>
#include <map>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <fmt/core.h>
#include <fmt/color.h>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>

#include <dependencies/watchdog_timer.hpp>

namespace dynamic_reconfigure_core
{
    enum ServiceWrapperReturnCodes
    {
        FAILURE = -1,
        SUCCESS = 0,
        BUSY
    };

    enum ServiceWrapperStates
    {
        IDLE = 0,
        PROCESSING,
        COMPLETE,
        TIMEOUT,
        ERROR
    };

    using AtomicServiceWrapperState = std::atomic<ServiceWrapperStates>;

    class ServiceWrapper
    {
    public:
        /// @brief Constructor for the ServiceWrapper class
        /// @param node
        ServiceWrapper(rclcpp::Node::SharedPtr node);

        /// @brief lists the available parameters of the node.
        /// @param node_name
        /// @return ServiceWrapperReturnCodes
        ServiceWrapperReturnCodes request_params_list(std::string node_name);

        /// @brief sets the parameters for the selected node
        /// @param params
        /// @return ServiceWrapperReturnCodes
        ServiceWrapperReturnCodes set_params(const std::vector<rclcpp::Parameter> &parameters);

        /// @brief get the parameters for the selected node
        /// @param param_names
        /// @return ServiceWrapperReturnCodes
        ServiceWrapperReturnCodes request_params(const std::vector<std::string> &param_names);

        /// @brief get the status of list retrieval co-routine
        /// @return ServiceWrapperStates
        ServiceWrapperStates get_list_status();

        /// @brief get the status of the request params co-routine
        /// @return ServiceWrapperStates
        ServiceWrapperStates get_request_status();

        /// @brief get the status of set params co-routine
        /// @return ServiceWrapperStates
        ServiceWrapperStates get_set_status();

        /// @brief get params list request before
        /// @return std::vector<std::string>
        std::vector<std::string> get_params_list();

        /// @brief get param types
        /// @return std::map<std::string, int>
        std::map<std::string, int> get_param_types();

        /// @brief get the requested params
        /// @return std::map<std::string, rcl_interfaces::msg::ParameterValue>
        std::map<std::string, rcl_interfaces::msg::ParameterValue> get_params();

        /// @brief clears all the error states
        void clear_errors();

        // TODO: write definition for this function.
        /// @brief reset all the states
        void reset_states();

    protected:
        /// @brief node name of the current node
        std::string node_name;

        /// @brief shared pointer to node
        rclcpp::Node::SharedPtr node_;

        /// @brief list params status state variable.
        AtomicServiceWrapperState list_params_status;

        /// @brief request params status state variable
        AtomicServiceWrapperState request_params_status;
        
        /// @brief set params status state variable
        AtomicServiceWrapperState set_params_status;

        /// @brief mutex to handle clients related resource sharing
        std::mutex client_mutex;

        /// @brief mutex to handle params related resource sharing
        std::mutex params_mutex;

        /// @brief mutex to handle request params resource sharing
        std::mutex requested_params_mutex;

        /// @brief parameters listed from the node
        std::vector<std::string> params;

        /// @brief parameters that are request from the node
        std::vector<std::string> requested_params;

        std::map<std::string, rcl_interfaces::msg::ParameterValue> retrieved_params;

        /// @brief holds the map for type of parameter for the given parameter
        std::map<std::string, int> parameter_types;

        /// @brief shared pointer to client that can set params
        rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_params_client_;

        /// @brief shared pointer to client that can get params
        rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;

        /// @brief shared pointer to client that can list params
        rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_params_client_;

        /// @brief shared pointer to client that can describe params
        rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_params_client_;

        /// @brief watchdog timer to monitor list parameters callback
        std::unique_ptr<dynamic_reconfigure_dependencies::WatchDogTimer> list_wd_timer;

        /// @brief watchdog timer to monitor get parameters callback
        std::unique_ptr<dynamic_reconfigure_dependencies::WatchDogTimer> get_wd_timer;

        /// @brief watchdog timer to monitor set parameters callback
        std::unique_ptr<dynamic_reconfigure_dependencies::WatchDogTimer> set_wd_timer;

        /// @brief creates the client to set, get or list params and it's types.
        /// @param node_name
        void create_client(const std::string node_name);

        /// @brief list the parameter types of the selected node
        void list_parameter_types();

    private:
        /// @brief this function is the callback for the list_params service call
        /// @param future
        void list_params_cb(const rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture future);

        /// @brief this function is the callback for the list_param_types service call
        /// @param future
        void list_param_types_cb(const rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture future);

        /// @brief this function is the callback for the set_params service call
        /// @param future
        void set_params_cb(const rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture future);

        /// @brief this function is the callback for the get_params service call
        /// @param future
        void request_params_cb(const rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future);
    };
};

#endif // SERVICE_WRAPPER_HPP__