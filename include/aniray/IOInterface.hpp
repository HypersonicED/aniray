/* IOInterface.hpp: Headers for IO for Aniray systems
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

#ifndef ANIRAY_IOINTERFACE_HPP
#define ANIRAY_IOINTERFACE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace aniray::IOInterface {

template <typename InputType>
class IOInterfaceInput {
  public:
    virtual auto getValues() const -> std::vector<InputType> {
        const std::shared_lock<std::shared_mutex> lock(mMutex);
        return mValues;
    }
    virtual auto hasValues() const -> bool {
        const std::shared_lock<std::shared_mutex> lock(mMutex);
        return !mValues.empty();
    }
    void setValues(std::vector<InputType> values) {
        const std::unique_lock<std::shared_mutex> lock(mMutex);
        mValues = std::move(values);
    }
    private:
        mutable std::shared_mutex mMutex;
        std::vector<InputType> mValues;
};

// NOTE: vector<bool> is space-efficient:
//       https://en.cppreference.com/w/cpp/container/vector_bool
using IOInterfaceInputDiscrete = IOInterfaceInput<bool>;

class IOInterfaceGeneric {
    public:
        [[nodiscard]] auto getInputsDiscrete(const std::string &name) const -> std::shared_ptr<IOInterfaceInputDiscrete>;
        virtual void refreshInputs() = 0;
    protected:
        void assignInputDiscrete(const std::string &name, std::shared_ptr<IOInterfaceInputDiscrete> input);
    private:
        std::unordered_map<std::string, std::shared_ptr<IOInterfaceInputDiscrete>> mInputsDiscrete;
};

} // namespace aniray::IOInterface

#endif // ANIRAY_IOINTERFACE_HPP
