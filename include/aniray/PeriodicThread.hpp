/* PeriodicThread.hpp: Headers for sampling Aniray systems
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
        void stop();
        [[nodiscard]] auto running() const -> bool;

        auto updateRate() const -> std::chrono::milliseconds;
        void updateRate(std::chrono::milliseconds updateRateMs);

    protected:
        virtual void periodicAction() = 0;
    private:
        bool mWasRunning = false;
        bool mRunning = false;
        std::chrono::milliseconds mUpdateRateMs;
        boost::asio::io_context mIOContext;
        std::unique_ptr<boost::asio::steady_timer> mTimer;
        std::unique_ptr<boost::thread> mIOThread;
        mutable std::shared_mutex mUpdateRateMutex;

        void timerHandler();
};

} // namespace aniray

#endif // ANIRAY_PERIODICTHREAD_HPP
