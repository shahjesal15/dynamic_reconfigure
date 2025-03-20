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
    }
    void setup()
    {
        service_wrapper = std::make_shared<dynamic_reconfigure_core::ServiceWrapper>(
            this->shared_from_this());
    }

    void timer_callback() {
        
        std::string name = "teleop_twist_keyboard";
        std::vector<std::string> params;

        service_wrapper->request_params_list(name);

        switch(service_wrapper->get_list_status()) {
            case dynamic_reconfigure_core::ServiceWrapperStates::COMPLETE:
                params = service_wrapper->get_params_list();
                for(std::string& param : params) {
                    RCLCPP_INFO_STREAM(this->get_logger(), param);
                }
                break;
            case dynamic_reconfigure_core::ServiceWrapperStates::ERROR:
                service_wrapper->clear_errors();
                break;
        };
    }

private:
    std::shared_ptr<dynamic_reconfigure_core::ServiceWrapper> service_wrapper;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceWrapperTest>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
}