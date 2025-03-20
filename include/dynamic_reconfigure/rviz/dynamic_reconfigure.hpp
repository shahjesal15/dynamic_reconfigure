#ifndef DYNAMIC_RECONFIGURE_HPP__
#define DYNAMIC_RECONFIGURE_HPP__

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rviz_common/panel.hpp>

#include <QDialog>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QCompleter>

#include <rviz/components/logger.hpp>
#include <dependencies/service_wrapper.hpp>

namespace dynamic_reconfigure
{
    class RvizDynamicReconfigure : public rviz_common::Panel
    {
    public:
        /// @brief constructor for the class
        /// @param parent
        RvizDynamicReconfigure(QWidget *parent = nullptr);

        /// @brief overriden function of the rviz_common::Panel class
        virtual void onInitialize() override;

        /// @brief destructor for the class
        ~RvizDynamicReconfigure();

        /// @brief init UI for the RViz2 plugin
        void init_ui();
    
    protected:
    
        /// @brief setup menubar for the panel
        void setup_menu();

        /// @brief setup widgets for any configurations required.
        void setup_widgets();

        /// @brief load configurations on startup or when calleds
        void load_configurations();

        /// @brief handle the button callbacks
        void handle_btns();
    
    private:   
        /// @brief ros2 node shared parameter
        rclcpp::Node::SharedPtr node_;

        /// @brief service wrapper code object ptr
        std::unique_ptr<dynamic_reconfigure_core::ServiceWrapper> service_wrapper;

        /// @brief combo boxes for node and param options
        QComboBox *node_options, *param_options; 

        /// @brief vertical layouts
        QVBoxLayout *reconfiguration_layout;

        /// @brief horizontal layouts
        QHBoxLayout *options_layout, *edit_layout;

        /// @brief push buttons
        QPushButton *set_btn, *get_btn;

        /// @brief line inputs
        QLineEdit *line_input;

        /// @brief slider to adjust the values of the params
        QSlider *param_slider;

        /// @brief menu bar for the params settings
        QMenuBar *menu_bar;
       
        /// @brief file menu for the menu bar 
        QMenu *file_menu;

        /// @brief action for refreshing the available nodes
        QAction *refresh_action, *exit_action;

        /// @brief log box for debugging and logging messages.
        QPlainTextEdit *log_box;

        /// @brief logger for displaying warnings, messages, etc.
        Logger *logger;
    };
}

#endif // DYNAMIC_RECONFIGURE_HPP__