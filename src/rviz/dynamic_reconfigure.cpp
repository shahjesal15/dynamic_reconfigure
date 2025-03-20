#include "rviz/dynamic_reconfigure.hpp"

namespace dynamic_reconfigure
{
    RvizDynamicReconfigure::RvizDynamicReconfigure(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        setWindowTitle("Dynamic Reconfigure");
        this->init_ui();
        this->setup_menu();
        this->setup_widgets();
    }

    void RvizDynamicReconfigure::onInitialize()
    {
        node_ = rclcpp::Node::make_shared("rviz_dynamic_reconfigure");
        service_wrapper = std::make_unique<dynamic_reconfigure_core::ServiceWrapper>(
            node_->shared_from_this());
        logger = new Logger(log_box);
    }

    void RvizDynamicReconfigure::handle_btns() {
        QObject *sender = QObject::sender();
        if (sender == set_btn) {
            logger->debug("set btn clicked");
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
        node_options->setEditable(true);
        node_options->setInsertPolicy(QComboBox::NoInsert);
        node_options->completer()->setCaseSensitivity(Qt::CaseInsensitive);
        node_options->completer()->setFilterMode(Qt::MatchContains);

        log_box->setReadOnly(true);

        param_options->setEditable(true);
        param_options->setInsertPolicy(QComboBox::NoInsert);
        param_options->completer()->setCaseSensitivity(Qt::CaseInsensitive);
        param_options->completer()->setFilterMode(Qt::MatchContains);

        QObject::connect(set_btn, &QPushButton::clicked, this, &RvizDynamicReconfigure::handle_btns);
        QObject::connect(get_btn, &QPushButton::clicked, this, &RvizDynamicReconfigure::handle_btns);
    }

    void RvizDynamicReconfigure::init_ui()
    {
        reconfiguration_layout = new QVBoxLayout();
        options_layout = new QHBoxLayout();
        edit_layout = new QHBoxLayout();

        node_options = new QComboBox();
        param_options = new QComboBox();

        line_input = new QLineEdit();

        param_slider = new QSlider(Qt::Horizontal);

        set_btn = new QPushButton("Set");
        get_btn = new QPushButton("Get");

        log_box = new QPlainTextEdit();

        options_layout->addWidget(node_options, 4);
        options_layout->addWidget(param_options, 6);

        edit_layout->addWidget(line_input, 4);
        edit_layout->addWidget(set_btn, 3);
        edit_layout->addWidget(get_btn, 3);

        reconfiguration_layout->addLayout(options_layout, 2);
        reconfiguration_layout->addWidget(param_slider, 1);
        reconfiguration_layout->addLayout(edit_layout, 3);
        reconfiguration_layout->addWidget(log_box, 4);

        setLayout(reconfiguration_layout);
    }

    RvizDynamicReconfigure::~RvizDynamicReconfigure() {}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dynamic_reconfigure::RvizDynamicReconfigure, rviz_common::Panel)