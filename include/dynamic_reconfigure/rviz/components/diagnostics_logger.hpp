#ifndef LOGGER_HPP__
#define LOGGER_HPP__

#include <queue>
#include <mutex>
#include <atomic>

#include <QObject>
#include <QPlainTextEdit>
#include <fmt/core.h>
#include <fmt/color.h>

namespace dynamic_reconfigure
{
    enum DiagnosticsLevel
    {
        ALL = 0,
        DEBUG,
        INFO,
        WARN,
        ERROR
    };

    struct DiagnosticsEntry {
        public:
            uint64_t message_id;
            DiagnosticsLevel urgency;
            std::string message;
        
        DiagnosticsEntry(); 
        
        DiagnosticsEntry(uint64_t id, DiagnosticsLevel level, std::string msg);
    };

    struct DiagnosticsCompare {
        bool operator()(const DiagnosticsEntry &a, const DiagnosticsEntry &b); 
    };

    class DiagnosticsLogger : public QObject
    {
        Q_OBJECT                     
    public:
        /// @brief constructor to the logger class
        /// @param log_box_
        explicit DiagnosticsLogger(QPlainTextEdit *log_box_, QObject *parent);

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
        void set_log_level(DiagnosticsLevel level);

        /// @brief updates all the pending logs
        void update_logs();

        /// @brief destructor to the logger class
        ~DiagnosticsLogger();

    protected:
        /// @brief log box object
        QPlainTextEdit *log_box;

        /// @brief log level
        DiagnosticsLevel log_level;

        /// @brief mutex for message queue
        std::mutex queue_mutex;

        /// @brief queue that maintain a priority queue
        std::priority_queue<DiagnosticsEntry, std::vector<DiagnosticsEntry>, DiagnosticsCompare> diagnostics_queue;

        /// @brief current message id
        uint64_t message_id;
    };
}
#endif // LOGGER_HPP__