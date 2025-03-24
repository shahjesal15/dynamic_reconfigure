#include "rviz/dynamic_reconfigure.hpp"

namespace dynamic_reconfigure
{
    RvizDynamicReconfigure::RvizDynamicReconfigure(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        node_name = "Dynamic Reconfigure";
        setWindowTitle(node_name);
        this->init_ui();

        logger = new DiagnosticsLogger(log_box, parent);

        this->setup_menu();
        this->setup_widgets();
    }

    void RvizDynamicReconfigure::onInitialize()
    {
        node_ = rclcpp::Node::make_shared("rviz_dynamic_reconfigure");
        service_wrapper = std::make_unique<dynamic_reconfigure_core::ServiceWrapper>(
            node_->shared_from_this());

        rate = std::make_shared<rclcpp::Rate>(100);

        executor_run.store(true);
        executor_thread = std::thread(&RvizDynamicReconfigure::update, this);
        executor_thread.detach();

        load_configurations();
    }

    void RvizDynamicReconfigure::list_nodes()
    {
        // Get the node graph interface
        auto node_graph = node_->get_node_graph_interface();

        // Get all node names and namespaces
        auto node_names = node_graph->get_node_names();

        for (std::string node_name : node_names)
        {
            node_name = (node_name.size() > 0 && node_name[0] == '/') ? node_name.substr(1) : node_name;
            if (node_options->findText(QString::fromStdString(node_name)) != -1)
                continue;
            QString item_name = QString::fromStdString(node_name);
            node_options->addItem(item_name);
        }

        if (node_names.size() != node_options->count())
        {
            for (uint16_t idx = 0; idx < node_options->count(); idx++)
            {
                if (std::find(node_names.begin(), node_names.end(), node_options->itemText(idx).toStdString()) == node_names.end())
                {
                    node_options->removeItem(idx);
                    logger->debug("certain non-existant node removed from options.");
                }
            }
            load_params();
        }
    }

    void RvizDynamicReconfigure::load_params()
    {
        std::string active_node = node_options->currentText().toStdString();

        param_options->setEnabled(false);
        line_input->setEnabled(false);
        set_btn->setEnabled(false);
        get_btn->setEnabled(false);
            
        if (active_node != "" && service_wrapper->request_params_list(active_node) == dynamic_reconfigure_core::ServiceWrapperReturnCodes::SUCCESS)
        {
            logger->debug("requested params from " + active_node);
        }
        else
        {

            logger->error("error occured while requesting params from " + active_node);
        }
    }

    void RvizDynamicReconfigure::handle_options(int index)
    {
        QObject *sender = QObject::sender();

        if (sender == node_options)
        {
            load_params();
        }
        else if (sender == param_options)
        {
            std::string param = param_options->currentText().toStdString();

            if(param.empty()) return;
            
            std::vector<std::string> requested_params = {param};
            service_wrapper->request_params(requested_params);

            logger->debug("requesting param : " + param);
        }
    }

    void RvizDynamicReconfigure::handle_btns()
    {
        QObject *sender = QObject::sender();
        if (sender == set_btn)
        {
            std::string param = param_options->currentText().toStdString();
            rclcpp::ParameterValue value;

            QString user_input = line_input->text();

            switch (param_types[param])
            {
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                value = rclcpp::ParameterValue(user_input.toInt());
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
                value = rclcpp::ParameterValue(user_input.toInt() == 1 ? true : false);
                break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                value = rclcpp::ParameterValue(user_input.toDouble());
                break;
            case rclcpp::ParameterType::PARAMETER_STRING:
                value = rclcpp::ParameterValue(user_input.toStdString());
                break;
            }

            std::vector<rclcpp::Parameter> to_set = {
                rclcpp::Parameter(param, value),
            };
            service_wrapper->set_params(to_set);
            logger->debug("updating param : " + param + " to " + user_input.toStdString());
           
            line_input->setEnabled(false);
            param_options->setEnabled(false);
            set_btn->setEnabled(false);
            get_btn->setEnabled(false);        

        }
        else if (sender == get_btn)
        {
            std::vector<std::string> requested_params = {param_options->currentText().toStdString()};
            service_wrapper->request_params(requested_params);
            logger->debug("requested param : " + requested_params[0]);
        }
    }

    void RvizDynamicReconfigure::handle_shortcuts()
    {
        QObject *sender = QObject::sender();
        QWidget *focused_widget = QApplication::focusWidget();

        if (sender == search_shortcut)
        {
            if (focused_widget == node_options  )
            {
                node_options->setEditable(true);
                node_options->setInsertPolicy(QComboBox::NoInsert);
                node_options->completer()->setCaseSensitivity(Qt::CaseInsensitive);
                node_options->completer()->setFilterMode(Qt::MatchContains);

                logger->debug("searching nodes");
            }
            else if (focused_widget == param_options)
            {
                param_options->setEditable(true);
                param_options->setInsertPolicy(QComboBox::NoInsert);
                param_options->completer()->setCaseSensitivity(Qt::CaseInsensitive);
                param_options->completer()->setFilterMode(Qt::MatchContains);

                logger->debug("searching params");
            }
        }
    }

    void RvizDynamicReconfigure::update()
    {
        while (executor_run.load() && rclcpp::ok())
        {
            rclcpp::spin_some(node_);

            RCLCPP_DEBUG_STREAM(node_->get_logger(), "executing reconfigure node.");

            if (service_wrapper->get_list_status() == dynamic_reconfigure_core::ServiceWrapperStates::COMPLETE)
            {
                param_options->clear();
                std::vector<std::string> params = service_wrapper->get_params_list();

                for (std::string param : params)
                {
                    param_options->addItem(QString::fromStdString(param));
                }
                param_types = service_wrapper->get_param_types();

                param_options->setCurrentIndex(0);
                param_options->setEnabled(false);

                logger->debug("refreshed params.");

                logger->update_logs();

                std::vector<std::string> requested_params = {param_options->currentText().toStdString()};
                service_wrapper->request_params(requested_params);
            }
            else if (service_wrapper->get_list_status() == dynamic_reconfigure_core::ServiceWrapperStates::ERROR)
            {
                param_options->clear();
                param_types.clear();
            }

            if (service_wrapper->get_request_status() == dynamic_reconfigure_core::ServiceWrapperStates::COMPLETE)
            {
                line_input->setEnabled(true);
                param_options->setEnabled(true);
                set_btn->setEnabled(true);
                get_btn->setEnabled(true);        

                std::string current_text = param_options->currentText().toStdString();
                QString place_holder = "", value = "";

                auto requested_params = service_wrapper->get_params();

                QRegularExpression regex = QRegularExpression("[01]");
                line_input->setValidator(nullptr);

                switch (requested_params[current_text].type)
                {
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    line_input->setValidator(new QIntValidator());
                    place_holder = "int";
                    value = QString::number(requested_params[current_text].integer_value);
                    break;
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    line_input->setValidator(new QRegularExpressionValidator(regex));
                    logger->warn("here");
                    value = QString::number(requested_params[current_text].bool_value);
                    place_holder = "bool";
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    line_input->setValidator(new QDoubleValidator());
                    place_holder = "double";
                    value = QString::number(requested_params[current_text].double_value);
                    break;
                case rclcpp::ParameterType::PARAMETER_STRING:
                    value = QString::fromStdString(requested_params[current_text].string_value);
                    place_holder = "string";
                    break;
                }

                line_input->setPlaceholderText(place_holder);
                line_input->setText(value);
            }
            else if (service_wrapper->get_request_status() == dynamic_reconfigure_core::ServiceWrapperStates::ERROR)
            {
                param_options->setEnabled(true);
                logger->error("failed to get certain params");
            }

            if (service_wrapper->get_set_status() == dynamic_reconfigure_core::ServiceWrapperStates::COMPLETE)
            {
                line_input->setEnabled(true);
                param_options->setEnabled(true);
                set_btn->setEnabled(true);
                get_btn->setEnabled(true);        
                
                logger->debug("certain params were set.");
            }
            rate->sleep();
        }
    }

    void RvizDynamicReconfigure::setup_menu()
    {
        menu_bar = new QMenuBar();

        file_menu = new QMenu("&File");

        refresh_action = new QAction("Refresh");
        exit_action = new QAction("Exit");

        file_menu->addAction(refresh_action);
        file_menu->addAction(exit_action);

        menu_bar->addMenu(file_menu);
        reconfiguration_layout->setMenuBar(menu_bar);
    }

    void RvizDynamicReconfigure::setup_widgets()
    {
        log_box->setReadOnly(true);

        node_options->installEventFilter(this);
        param_options->installEventFilter(this);

        QObject::connect(set_btn, &QPushButton::clicked, this, &RvizDynamicReconfigure::handle_btns);
        QObject::connect(get_btn, &QPushButton::clicked, this, &RvizDynamicReconfigure::handle_btns);
        QAction::connect(refresh_action, &QAction::triggered, this, &RvizDynamicReconfigure::list_nodes);

        QObject::connect(node_options, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &RvizDynamicReconfigure::handle_options);
        QObject::connect(param_options, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &RvizDynamicReconfigure::handle_options);

        QObject::connect(search_shortcut, &QShortcut::activated, this, &RvizDynamicReconfigure::handle_shortcuts);
    }

    bool RvizDynamicReconfigure::event(QEvent *event)
    {
        if (event->type() == QEvent::Hide)
        {
            executor_run.store(false);
            this->deleteLater();
        }
        return rviz_common::Panel::event(event);
    }

    bool RvizDynamicReconfigure::eventFilter(QObject *obj, QEvent *event)
    {
        if (event->type() == QEvent::FocusOut)
        {
            if (obj == node_options)
            {
                node_options->setEditable(false);
                logger->debug("lost focus from node options");
            }
            else if (obj == param_options)
            {
                param_options->setEditable(false);
                logger->debug("lost focus from param options");
            }
        }
        return QObject::eventFilter(obj, event);
    }

    void RvizDynamicReconfigure::load_configurations()
    {
        list_nodes();
    }

    void RvizDynamicReconfigure::init_ui()
    {
        reconfiguration_layout = new QVBoxLayout();
        options_layout = new QHBoxLayout();
        edit_layout = new QHBoxLayout();

        node_options = new QComboBox();
        param_options = new QComboBox();

        line_input = new QLineEdit();

        set_btn = new QPushButton("Set");
        get_btn = new QPushButton("Get");

        log_box = new QPlainTextEdit();

        search_shortcut = new QShortcut(QKeySequence("Alt+S"), this);

        options_layout->addWidget(node_options, 4);
        options_layout->addWidget(param_options, 6);

        edit_layout->addWidget(line_input, 4);
        edit_layout->addWidget(set_btn, 3);
        edit_layout->addWidget(get_btn, 3);

        reconfiguration_layout->addLayout(options_layout, 3);
        reconfiguration_layout->addLayout(edit_layout, 3);
        reconfiguration_layout->addWidget(log_box, 6);

        setLayout(reconfiguration_layout);
    }

    RvizDynamicReconfigure::~RvizDynamicReconfigure() {}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dynamic_reconfigure::RvizDynamicReconfigure, rviz_common::Panel)