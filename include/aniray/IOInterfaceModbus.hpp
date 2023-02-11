/* InputInterfaceModbus.hpp: Headers for modbus inputs to Aniray systems
 *
 * Created by Perry Naseck on 2022-11-04.
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

#ifndef ANIRAY_INPUTINTERFACEMODBUS_HPP
#define ANIRAY_INPUTINTERFACEMODBUS_HPP

#include <cstdint>
#include <shared_mutex>
#include <string>
#include <vector>

#include <modbus/modbus.h>

#include <aniray/IOInterface.hpp>

namespace aniray {
namespace IOInterface {
namespace Modbus {

enum class ConfigFunctionsAddressLayout {
    ADDRESS, // each address is an input
    BITS_LSB, // each bit is an input, first bit is first address
    SPAN_2_LSB // 16- or 32-bit value over two addresses, LSB
};
struct ConfigInputDiscrete {
  std::string name;
  std::uint8_t slaveID;
  std::uint8_t functionCode;
  ConfigFunctionsAddressLayout addressLayout;
  std::uint16_t startAddress;
  std::uint16_t numAddressedItems;
  bool enableClear;
  std::uint8_t clearFunctionCode;
  bool clearSingleAddress;
  std::uint16_t clearAddress;
//   std::uint16_t clearNumAddressedItems;
};

// Not enum to enforce strong typing
const std::uint8_t FUNCTION_CODE_READ_BITS = 1;
const std::uint8_t FUNCTION_CODE_READ_INPUT_BITS = 2;
const std::uint8_t FUNCTION_CODE_READ_REGISTERS = 3;
const std::uint8_t FUNCTION_CODE_READ_INPUT_REGISTERS = 4;
const std::uint8_t FUNCTION_CODE_FORCE_SINGLE_COIL = 5;

class IOInterfaceModbus : public aniray::IOInterface::IOInterfaceGeneric {
    public:
        IOInterfaceModbus(std::string tcpAddress, std::uint16_t tcpPort);
        ~IOInterfaceModbus();
        void refreshInputs() override;
        void setupInputDiscrete(std::string name,
                                std::uint8_t slaveID,
                                std::uint8_t functionCode,
                                ConfigFunctionsAddressLayout addressLayout,
                                std::uint16_t startAddress,
                                std::uint16_t numAddressedItems,
                                bool enableClear = false,
                                std::uint8_t clearFunctionCode = FUNCTION_CODE_FORCE_SINGLE_COIL,
                                bool clearSingleAddress = false,
                                std::uint16_t clearAddress = 0);
    private:
        std::unordered_map<std::string, ConfigInputDiscrete> mInputsDiscreteModbus;
        mutable std::shared_mutex mMutexInputsDiscreteModbus;
        modbus_t *mCTX;

        void setupConnectionTCP(std::string tcpAddress, std::uint16_t tcpPort);
        void updateInputDiscrete(ConfigInputDiscrete configInputDiscrete);

        // void updateInputsCounter(ConfigFunction functionConfig) {}
};

} // namespace IOInterfaceModbus
} // namespace IOInterface
} // namespace aniray

#endif // ANIRAY_INPUTINTERFACEMODBUS_HPP
