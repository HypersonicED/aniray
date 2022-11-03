/* PeriodicThread.hpp: Headers for sampling Aniray systems
 *
 * Created by Perry Naseck on 2022-11-03.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_PERIODICTHREAD_HPP
#define ANIRAY_PERIODICTHREAD_HPP

#include <chrono>
#include <memory>
#include <shared_mutex>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/thread.hpp>

namespace aniray {

class PeriodicThread { // NOLINT(cppcoreguidelines-special-member-functions,hicpp-special-member-functions)
    public:
        PeriodicThread(std::chrono::milliseconds updateRateMs);
        ~PeriodicThread();
        PeriodicThread(PeriodicThread&) = delete; // copy constructor
        PeriodicThread(const PeriodicThread&) = delete; // copy constructor
        PeriodicThread(PeriodicThread&&) = delete;  // move constructor
        auto operator=(PeriodicThread&) -> PeriodicThread& = delete;  // copy assignment
        auto operator=(const PeriodicThread&) -> PeriodicThread& = delete;  // copy assignment
        // auto operator=(PeriodicThread&&) noexcept -> PeriodicThread& = default;  // move assignment
        // move assignment implicitly deleted by boost::asio::io_context (-Wdefaulted-function-deleted)

        void start();

        auto updateRate() const -> std::chrono::milliseconds;
        void updateRate(std::chrono::milliseconds updateRateMs);

    protected:
        virtual void periodicAction() = 0;
    private:
        std::chrono::milliseconds mUpdateRateMs;
        boost::asio::io_context mIOContext;
        std::unique_ptr<boost::asio::steady_timer> mTimer;
        std::unique_ptr<boost::thread> mIOThread;
        mutable std::shared_mutex mUpdateRateMutex;

        void timerHandler ();
};

} // namespace aniray

#endif // ANIRAY_PERIODICTHREAD_HPP
