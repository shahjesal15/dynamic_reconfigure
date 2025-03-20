#include "rviz/components/logger.hpp"

namespace dynamic_reconfigure {
    Logger::Logger(QPlainTextEdit *log_box_) : log_box(log_box_) {}

    void Logger::error(std::string message) {
        std::string buffer = "<p><span style=\"color: red; font-weight: bold;\">error </span>" + message + "</p>";        
        log_box->appendHtml(QString::fromStdString(buffer));
    }

    void Logger::info(std::string message) {
        std::string buffer = "<p><span style=\"color: blue; font-weight: bold;\">info </span>" + message + "</p>";        
        log_box->appendHtml(QString::fromStdString(buffer));
    }

    void Logger::debug(std::string message) {
        std::string buffer = "<p><span style=\"color: green; font-weight: bold;\">debug </span>" + message + "</p>";        
        log_box->appendHtml(QString::fromStdString(buffer));
    }

    void Logger::warn(std::string message) {
        std::string buffer = "<p><span style=\"color: #FBB117; font-weight: bold;\">warning </span>" + message + "</p>";        
        log_box->appendHtml(QString::fromStdString(buffer));
    }

    Logger::~Logger() {}
}