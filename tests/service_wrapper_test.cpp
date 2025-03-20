#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "dependencies/service_wrapper.hpp"

using namespace std::chrono_literals; 

class ServiceWrapperTest : public rclcpp::Node
{
public:
    ServiceWrapperTest() : Node("service_wrapper_test") {
        timer = this->create_wall_timer(5ms, std::bind(&ServiceWrapperTest::timer_callback, this));
        
        this->declare_parameter("param1", test[0]);
        this->declare_parameter("param2", test[1]);
        this->declare_parameter("param3", test[2]);

        std::vector<std::string> params = {"param1"};
    }
    void setup()
    {
        service_wrapper = std::make_shared<dynamic_reconfigure_core::ServiceWrapper>(
            this->shared_from_this());
    }

    void timer_callback() {
        std::string name = "service_wrapper_test";
        std::vector<std::string> params = {};
        std::map<std::string, rcl_interfaces::msg::ParameterValue> retrieved_params;
        
        service_wrapper->request_params_list(name);

        switch(service_wrapper->get_list_status()) {
            case dynamic_reconfigure_core::ServiceWrapperStates::COMPLETE:
                params = service_wrapper->get_params_list();
                service_wrapper->request_params(params);
                break;
            case dynamic_reconfigure_core::ServiceWrapperStates::ERROR:
                service_wrapper->clear_errors();
                break;
        };
        std::string param_name = "param1";
        rclcpp::ParameterValue param_value(test[0]++);

        switch(service_wrapper->get_request_status()) {
            case dynamic_reconfigure_core::ServiceWrapperStates::COMPLETE:
                retrieved_params = service_wrapper->get_params();
                for(auto &param : retrieved_params) {
                    RCLCPP_INFO_STREAM(this->get_logger(),
                        fmt::format(fg(fmt::color::fuchsia), "{} {}", param.first, param.second.double_value));
                }
                service_wrapper->set_params({rclcpp::Parameter(param_name, param_value)});
                break;
            case dynamic_reconfigure_core::ServiceWrapperStates::ERROR:
                service_wrapper->clear_errors();
                break;
        }
        switch(service_wrapper->get_set_status()) {
            case dynamic_reconfigure_core::ServiceWrapperStates::COMPLETE:
                RCLCPP_INFO_STREAM(this->get_logger(),
                    fmt::format(fg(fmt::color::wheat), "parameter set"));
            case dynamic_reconfigure_core::ServiceWrapperStates::ERROR:
                service_wrapper->clear_errors();
                break;    
        }
    }

private:
    std::shared_ptr<dynamic_reconfigure_core::ServiceWrapper> service_wrapper;
    rclcpp::TimerBase::SharedPtr timer;
    double test[3] = {1, 2, 3};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceWrapperTest>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
}