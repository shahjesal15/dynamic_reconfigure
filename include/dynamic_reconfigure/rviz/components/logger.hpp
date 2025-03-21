#ifndef LOGGER_HPP__
#define LOGGER_HPP__

#include <QObject>
#include <QPlainTextEdit>
#include <fmt/core.h>
#include <fmt/color.h>

namespace dynamic_reconfigure
{
    enum LoggingLevel
    {
        ALL = 0,
        DEBUG,
        INFO,
        WARN,
        ERROR
    };

    class Logger : public QObject
    {
        Q_OBJECT                     
    public:
        /// @brief constructor to the logger class
        /// @param log_box_
        explicit Logger(QPlainTextEdit *log_box_, QObject *parent);

        /// @brief display the error message
        /// @param message
        void error(std::string message);

        /// @brief display the info message
        /// @param message
        void info(std::string message);

        /// @brief display the debug message
        /// @param message
        void debug(std::string message);

        /// @brief display the warnings
        /// @param message
        void warn(std::string message);

        /// @brief set log level
        /// @param level
        void set_log_level(LoggingLevel level);

        /// @brief destructor to the logger class
        ~Logger();

    protected:
        /// @brief log box object
        QPlainTextEdit *log_box;

        /// @brief log level
        LoggingLevel log_level;

        /// @brief append thread safe messages
        /// @param message
        void appendLog(const QString &message);

    signals:
        void logMessage(QString message);
    };
}
#endif // LOGGER_HPP__