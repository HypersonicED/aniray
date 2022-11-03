/* PeriodicThread.cpp: Headers for sampling Aniray systems
 *
 * Created by Perry Naseck on 2022-11-03.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#include <chrono>
// #include <iostream>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp> // IWYU pragma: keep
#include <boost/thread.hpp> // IWYU pragma: keep
// IWYU pragma: no_include <boost/asio/basic_waitable_timer.hpp>
// IWYU pragma: no_include <boost/thread/thread_only.hpp>
// IWYU pragma: no_forward_declare boost::system::error_code

#include <aniray/PeriodicThread.hpp>

namespace aniray {

PeriodicThread::PeriodicThread(std::chrono::milliseconds updateRateMs)
    : mUpdateRateMs(updateRateMs) {
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard(mIOContext.get_executor());
    mTimer = std::make_unique<boost::asio::steady_timer>(
        mIOContext,
        std::chrono::steady_clock::now() + mUpdateRateMs);
    mTimer->async_wait([this] (const boost::system::error_code&) { timerHandler(); });
}

void PeriodicThread::start() {
    mIOThread = std::make_unique<boost::thread>([ObjectPtr = &mIOContext] { ObjectPtr->run(); });
}

auto PeriodicThread::updateRate() const -> std::chrono::milliseconds {
    const std::shared_lock<std::shared_mutex> lock(mUpdateRateMutex);
    return mUpdateRateMs;
}

void PeriodicThread::updateRate(std::chrono::milliseconds updateRateMs) {
    const std::unique_lock<std::shared_mutex> lock(mUpdateRateMutex);
    mUpdateRateMs = updateRateMs;
}

PeriodicThread::~PeriodicThread() {
    mIOContext.stop();
    // if (mIOThread->joinable()) {
    //     mIOThread->join();
    // }
}

void PeriodicThread::timerHandler () {
    periodicAction();
    const std::shared_lock<std::shared_mutex> lock(mUpdateRateMutex);
    mTimer->expires_at(mTimer->expiry() + mUpdateRateMs);
    mTimer->async_wait([this] (const boost::system::error_code&) { timerHandler(); });
}

} // namespace aniray
