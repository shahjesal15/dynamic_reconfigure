#ifndef WATCHDOG_TIMER_HPP__
#define WATCHDOG_TIMER_HPP__

#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <mutex>
#include <condition_variable>

namespace dynamic_reconfigure_dependencies
{
    enum WatchDogStatus {
        ACTIVE = 0,
        TIMEOUT,
        STOPPED
    };

    class WatchDogTimer
    {
    public:
        /// @brief WatchDogTimer constructor
        /// @param millis
        /// @param callback
        WatchDogTimer(std::string wd_name, unsigned int interval, std::function<void()> callback);

        /// @brief stops the watch dog timer
        void stop();

        /// @brief restarts the watchdog timer
        void restart();
        
        /// @brief get timer status
        /// @return WatchDogStatus
        WatchDogStatus get_status();

        /// @brief destructor for the WatchDogTimer
        ~WatchDogTimer();

    protected:
        /// @brief watchdog timer name
        std::string wd_name; 
        
        /// @brief watchdog thread that monitors the timeout 
        std::thread watchdog_thread;
        
        /// @brief interval for the timer to sleep
        unsigned int interval;
        
        /// @brief active status for the watchdog timer
        std::atomic<WatchDogStatus> active_status;

        /// @brief callback after the timer expires
        std::function<void()> callback;

        /// @brief mutex for the watch dog timer thread
        std::mutex mutex;

        /// @brief condtion variable to notify the thread
        std::condition_variable cv;

        /// @brief init the watchdog timer
        void init();

        /// @brief monitor for the timer to end or the be ended
        void monitor();
    };
}

#endif // WATCHDOG_TIMER_HPP__