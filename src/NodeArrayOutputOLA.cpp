/* NodeArrayOutputOLA.cpp: Aniray system OLA output
 *
 * Created by Perry Naseck on 2022-09-22.
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

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ola/Callback.h>
#include <ola/Clock.h>
#include <ola/DmxBuffer.h>
#include <ola/client/ClientArgs.h>
#include <ola/client/ClientWrapper.h>
#include <ola/client/OlaClient.h>
#include <ola/io/SelectServer.h>
#include <ola/thread/Thread.h>

#include <aniray/NodeArrayOutputOLA.hpp>

namespace aniray {

NodeArrayOutputOLAThread::NodeArrayOutputOLAThread(const Options &options)
    : ola::thread::Thread(options) {}

auto NodeArrayOutputOLAThread::Start(const ola::TimeInterval &period,
            std::unordered_map<std::uint32_t, std::size_t> &universesToBuffers) -> bool {
    mPeriod = period;
    mUniversesToBuffers = universesToBuffers;
    if (!mOLAClientWrapper.Setup()) {
    return false;
    }
    return ola::thread::Thread::Start();
}

void NodeArrayOutputOLAThread::Stop() {
    mOLAClientWrapper.GetSelectServer()->Terminate();
}

auto NodeArrayOutputOLAThread::GetSelectServer() -> ola::io::SelectServer* {
    return mOLAClientWrapper.GetSelectServer();
}

void NodeArrayOutputOLAThread::updateData(std::vector<ola::DmxBuffer> buffers) {
    const std::lock_guard<std::mutex> lock(mUpdateMutex);
    mBuffers = std::move(buffers);
}

auto NodeArrayOutputOLAThread::Run() -> void* {
    mOLAClientWrapper.GetSelectServer()->RegisterRepeatingTimeout(
    mPeriod,
    ola::NewCallback(
        this,
        &NodeArrayOutputOLAThread::InternalSendUniverses));
    mOLAClientWrapper.GetSelectServer()->Run();
    return nullptr;
}

auto NodeArrayOutputOLAThread::InternalSendUniverses() -> bool {
    auto *olaClient = mOLAClientWrapper.GetClient();
    const std::lock_guard<std::mutex> lock(mUpdateMutex);
    for (auto const &[universe, i] : mUniversesToBuffers) {
    if (i >= mBuffers.size()) {
        continue;
    }
    olaClient->SendDMX(universe, mBuffers[i], ola::client::SendDMXArgs());
    }
    return true;
}

} // namespace aniray

