#include <dependencies/watchdog_timer.hpp>

namespace dynamic_reconfigure_dependencies
{
    WatchDogTimer::WatchDogTimer(std::string wd_name, unsigned int interval, std::function<void()> callback)
        : wd_name(wd_name), interval(interval), callback(callback)
    {
        init();
    }

    void WatchDogTimer::init()
    {
        watchdog_thread = std::thread(&WatchDogTimer::monitor, this);
        active_status.store(WatchDogStatus::ACTIVE);
    }

    void WatchDogTimer::monitor()
    {
        std::unique_lock<std::mutex> lock(mutex);

        if (cv.wait_for(lock, std::chrono::milliseconds(interval)) == std::cv_status::no_timeout)
        {
            std::cout << wd_name << " stopped by condition" << std::endl;
            active_status.store(WatchDogStatus::STOPPED);
        }
        else
        {
            std::cout << wd_name << " stopped by timeout" << std::endl;
            active_status.store(WatchDogStatus::TIMEOUT);
            callback();
        }
    }

    void WatchDogTimer::stop()
    {
        if (active_status.load() == WatchDogStatus::ACTIVE)
        {
            cv.notify_one();
        }

        if (watchdog_thread.joinable())
        {
            watchdog_thread.join();
        }
    }

    void WatchDogTimer::restart()
    {
        stop();
        init();
    }

    WatchDogStatus WatchDogTimer::get_status()
    {
        return active_status.load();
    }

    WatchDogTimer::~WatchDogTimer()
    {
        stop();
    }
};