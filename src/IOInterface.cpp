/* IOInterface.cpp: IO for Aniray systems
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

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <aniray/IOInterface.hpp>

namespace aniray::IOInterface {

auto IOInterfaceGeneric::getInputsDiscrete(const std::string &name) const -> std::shared_ptr<IOInterfaceInputDiscrete> {
    return mInputsDiscrete.at(name);
}

void IOInterfaceGeneric::assignInputDiscrete(const std::string &name, std::shared_ptr<IOInterfaceInputDiscrete> input) {
    if (mInputsDiscrete.count(name) > 0) {
        throw std::runtime_error("IOInterfaceGeneric: Duplicate discrete input! Name: " + name);
    }
    mInputsDiscrete[name] = std::move(input);
}

} // namespace aniray::IOInterface
