/* PeriodicThread.cpp: Headers for sampling Aniray systems
 *
 * Created by Perry Naseck on 2022-11-03.
 *
 * This file is a part of Aniray
 * https://github.com/HypersonicED/aniray
 *
 * Copyright (c) 2022, Hypersonic
 * Copyright (c) 2022, Perry Naseck
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
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
    mRunning = true;
    mIOThread = std::make_unique<boost::thread>([ObjectPtr = &mIOContext] { ObjectPtr->run(); });
}

void PeriodicThread::stop() {
    mIOContext.stop();
    mRunning = false;
}

[[nodiscard]] auto PeriodicThread::running() const -> bool {
    return mRunning;
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
   stop();
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
