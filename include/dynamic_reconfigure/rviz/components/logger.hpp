#ifndef LOGGER_HPP__
#define LOGGER_HPP__

#include <QPlainTextEdit>
#include <fmt/core.h>
#include <fmt/color.h>

namespace dynamic_reconfigure
{
    class Logger
    {
        public:
            /// @brief constructor to the logger class
            /// @param log_box_ 
            Logger(QPlainTextEdit *log_box_);

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

            /// @brief destructor to the logger class
            ~Logger();
        protected:
            /// @brief log box object
            QPlainTextEdit *log_box;
    };
}
#endif // LOGGER_HPP__