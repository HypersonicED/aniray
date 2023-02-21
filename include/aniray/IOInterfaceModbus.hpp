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

#ifndef ANIRAY_IOINTERFACEMODBUS_HPP
#define ANIRAY_IOINTERFACEMODBUS_HPP

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <modbus/modbus.h>

#include <aniray/IOInterface.hpp>
#include <aniray/PeriodicThread.hpp>

namespace aniray::IOInterface::Modbus {

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
  std::uint16_t clearStartAddress;
  bool onlyClearSingleAddress;
  bool clearHigh;
//   std::uint16_t clearNumAddressedItems;
};

// Not enum to enforce strong typing
const std::uint8_t FUNCTION_CODE_READ_BITS = 1;
const std::uint8_t FUNCTION_CODE_READ_INPUT_BITS = 2;
const std::uint8_t FUNCTION_CODE_READ_REGISTERS = 3;
const std::uint8_t FUNCTION_CODE_READ_INPUT_REGISTERS = 4;
const std::uint8_t FUNCTION_CODE_FORCE_SINGLE_COIL = 5;
const std::uint8_t FUNCTION_CODE_FORCE_MULTIPLE_COILS = 15;
const std::uint8_t FUNCTION_CODE_PRESET_SINGLE_REGISTERS = 6;
const std::uint8_t FUNCTION_CODE_PRESET_MULTIPLE_REGISTERS = 16;

const std::size_t MODBUS_BITS_PER_BYTE = 8;
const std::size_t MODBUS_BITS_PER_REGISTER = 16;

const std::uint8_t CLEAR_BIT_VALUE_LOW = 0;
const std::uint16_t CLEAR_REGISTER_VALUE_LOW = 0;
const std::uint8_t CLEAR_BIT_VALUE_HIGH = 1;
const std::uint16_t CLEAR_REGISTER_VALUE_HIGH = 1;

class IOInterfaceModbus : public aniray::IOInterface::IOInterfaceGeneric {
    public:
        IOInterfaceModbus(std::string tcpAddress, std::uint16_t tcpPort);
        ~IOInterfaceModbus();

        IOInterfaceModbus(IOInterfaceModbus&) = delete; // copy constructor
        IOInterfaceModbus(const IOInterfaceModbus&) = delete; // copy constructor
        IOInterfaceModbus(IOInterfaceModbus&&) = delete;  // move constructor
        auto operator=(IOInterfaceModbus&) -> IOInterfaceModbus& = delete;  // copy assignment
        auto operator=(const IOInterfaceModbus&) -> IOInterfaceModbus& = delete;  // copy assignment
        auto operator=(IOInterfaceModbus&&) noexcept -> IOInterfaceModbus& = delete;  // move assignment

        void refreshInputs() override;
        void setupInputDiscrete(const std::string &name,
                                std::uint8_t slaveID,
                                std::uint8_t functionCode,
                                ConfigFunctionsAddressLayout addressLayout,
                                std::uint16_t startAddress,
                                std::uint16_t numAddressedItems);
        void setupInputDiscrete(const std::string &name,
                                std::uint8_t slaveID,
                                std::uint8_t functionCode,
                                ConfigFunctionsAddressLayout addressLayout,
                                std::uint16_t startAddress,
                                std::uint16_t numAddressedItems,
                                std::uint8_t clearFunctionCode,
                                std::uint16_t clearStartAddress,
                                bool onlyClearSingleAddress,
                                bool clearHigh);
    private:
        std::unordered_map<std::string, ConfigInputDiscrete> mInputsDiscreteModbus;
        mutable std::shared_mutex mMutexInputsDiscreteModbus;
        modbus_t *mCTX;

        void setupConnectionTCP(std::string tcpAddress, std::uint16_t tcpPort);
        void updateInputDiscreteModbusRead(const ConfigInputDiscrete &configInputDiscrete,
                                           std::vector<std::uint8_t> &dest8,
                                           std::vector<std::uint16_t> &dest16);
        void updateInputDiscreteModbusClear(const ConfigInputDiscrete &configInputDiscrete);
        static void updateInputDiscreteModbusTransformAddress(const ConfigInputDiscrete &configInputDiscrete,
                                                              std::vector<std::uint8_t> &dest8,
                                                              std::vector<std::uint16_t> &dest16,
                                                              std::vector<bool> &out);
        static void updateInputDiscreteModbusTransformBitsLSB(const ConfigInputDiscrete &configInputDiscrete,
                                                              std::vector<std::uint8_t> &dest8,
                                                              std::vector<std::uint16_t> &dest16,
                                                              std::vector<bool> &out);
        static void updateInputDiscreteModbusTransformSpan2LSB(const ConfigInputDiscrete &configInputDiscrete,
                                                               std::vector<std::uint8_t> &dest8,
                                                               std::vector<std::uint16_t> &dest16,
                                                               std::vector<bool> &out);
        void updateInputDiscrete(const ConfigInputDiscrete &configInputDiscrete);
        void setupInputDiscreteNoLock(const std::string &name,
                                      std::uint8_t slaveID,
                                      std::uint8_t functionCode,
                                      ConfigFunctionsAddressLayout addressLayout,
                                      std::uint16_t startAddress,
                                      std::uint16_t numAddressedItems);

        // void updateInputsCounter(ConfigFunction functionConfig) {}
};

class IOInterfaceModbusPeriodicThread : public IOInterfaceModbus, public PeriodicThread {
    public:
        IOInterfaceModbusPeriodicThread(std::string tcpAddress, std::uint16_t tcpPort, std::chrono::milliseconds updateRateMs);
        auto refreshHasErrored() -> bool;
    private:
        void periodicAction() override;
        std::atomic<bool> mRefreshError = false;
};

} // namespace aniray::IOInterface::Modbus

#endif // ANIRAY_IOINTERFACEMODBUS_HPP
