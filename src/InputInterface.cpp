/* InputInterface.cpp: Inputs to Aniray systems
 *
 * Created by Perry Naseck on 2022-11-03.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
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
