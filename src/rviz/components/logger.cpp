#include "rviz/components/logger.hpp"

namespace dynamic_reconfigure {
    Logger::Logger(QPlainTextEdit *log_box_, QObject *parent) : log_box(log_box_), QObject(parent) {
        set_log_level(LoggingLevel::DEBUG);
        QObject::connect(this, &Logger::logMessage, this, &Logger::appendLog, Qt::QueuedConnection);
    }

    void Logger::appendLog(const QString &message) {
        if (log_box) {
            log_box->appendHtml(message);  // UI update happens in the main thread
        }
    }

    void Logger::error(std::string message) {
        if(log_level > LoggingLevel::ERROR)
            return;

        std::string buffer = "<p><span style=\"color: red; font-weight: bold;\">error </span>" + message + "</p>";        
        emit logMessage(QString::fromStdString(buffer));
    }

    void Logger::info(std::string message) {
        if(log_level > LoggingLevel::INFO)
            return;

        std::string buffer = "<p><span style=\"color: blue; font-weight: bold;\">info </span>" + message + "</p>";        
        emit logMessage(QString::fromStdString(buffer));
    }

    void Logger::debug(std::string message) {
        if(log_level > LoggingLevel::DEBUG)
            return;

        std::string buffer = "<p><span style=\"color: green; font-weight: bold;\">debug </span>" + message + "</p>";        
        emit logMessage(QString::fromStdString(buffer));        
    }

    void Logger::warn(std::string message) {
        if(log_level > LoggingLevel::WARN)
            return;

        std::string buffer = "<p><span style=\"color: #FBB117; font-weight: bold;\">warning </span>" + message + "</p>";        
        emit logMessage(QString::fromStdString(buffer));        
    }

    void Logger::set_log_level(LoggingLevel log_level) {
        this->log_level = log_level;
    }

    Logger::~Logger() {}
}