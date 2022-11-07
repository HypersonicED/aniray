/* InputInterface.cpp: Inputs to Aniray systems
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

#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <vector>

#include <aniray/InputInterface.hpp>

namespace aniray {

auto InputInterface::getInputs() const -> std::vector<std::uint16_t> {
    const std::shared_lock<std::shared_mutex> lock(mInputsMutex);
    return mInputs;
}

void InputInterface::setInputs(std::vector<std::uint16_t> inputs) {
    const std::unique_lock<std::shared_mutex> lock(mInputsMutex);
    mInputs = std::move(inputs);
}

} // namespace aniray
