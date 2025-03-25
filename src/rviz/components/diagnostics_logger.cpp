#include "rviz/components/diagnostics_logger.hpp"

namespace dynamic_reconfigure
{
    DiagnosticsEntry::DiagnosticsEntry() {}

    DiagnosticsEntry::DiagnosticsEntry(uint64_t id, DiagnosticsLevel level, std::string msg)
        : message_id(id), urgency(level), message(msg)
    {
    }

    bool DiagnosticsCompare::operator()(const DiagnosticsEntry &a, const DiagnosticsEntry &b)
    {
        if (a.urgency != b.urgency)
            return a.urgency < b.urgency;
        return a.message_id > b.message_id;
    }

    DiagnosticsLogger::DiagnosticsLogger(QPlainTextEdit *log_box_, QObject *parent) : log_box(log_box_), QObject(parent)
    {
        qRegisterMetaType<QTextCursor>("QTextCursor");
        set_log_level(DiagnosticsLevel::DEBUG);
        message_id = 0;
    }

    void DiagnosticsLogger::error(std::string message)
    {
        if (log_level > DiagnosticsLevel::ERROR)
            return;

        std::lock_guard<std::mutex> lock(queue_mutex);

        std::string buffer = "<p><span style=\"color: red; font-weight: bold;\">error </span>" + message + "</p>";
        diagnostics_queue.push(DiagnosticsEntry(message_id++, DiagnosticsLevel::ERROR, buffer));
    }

    void DiagnosticsLogger::info(std::string message)
    {
        if (log_level > DiagnosticsLevel::INFO)
            return;

        std::lock_guard<std::mutex> lock(queue_mutex);

        std::string buffer = "<p><span style=\"color: blue; font-weight: bold;\">info </span>" + message + "</p>";
        diagnostics_queue.push(DiagnosticsEntry(message_id++, DiagnosticsLevel::INFO, buffer));

    }

    void DiagnosticsLogger::debug(std::string message)
    {
        if (log_level > DiagnosticsLevel::DEBUG)
            return;

        std::lock_guard<std::mutex> lock(queue_mutex);

        std::string buffer = "<p><span style=\"color: green; font-weight: bold;\">debug </span>" + message + "</p>";
        diagnostics_queue.push(DiagnosticsEntry(message_id++, DiagnosticsLevel::DEBUG, buffer));        
    }

    void DiagnosticsLogger::warn(std::string message)
    {
        if (log_level > DiagnosticsLevel::WARN)
            return;

        std::lock_guard<std::mutex> lock(queue_mutex);

        std::string buffer = "<p><span style=\"color: #FBB117; font-weight: bold;\">warning </span>" + message + "</p>";
        diagnostics_queue.push(DiagnosticsEntry(message_id++, DiagnosticsLevel::WARN, buffer));
        
    }

    void DiagnosticsLogger::update_logs() {
        QMetaObject::invokeMethod(log_box, [this]() {
            std::lock_guard<std::mutex> lock(queue_mutex);
            while(!diagnostics_queue.empty()) {
                auto log = diagnostics_queue.top();
                log_box->appendHtml(QString::fromStdString(log.message));
                diagnostics_queue.pop();
            }
        }, Qt::QueuedConnection);
    }

    void DiagnosticsLogger::set_log_level(DiagnosticsLevel log_level)
    {
        this->log_level = log_level;
    }

    DiagnosticsLogger::~DiagnosticsLogger() {}
}