/* IOInterfaceModbus.cpp: Modbus IO for Aniray systems
 *
 * Created by Perry Naseck on 2023-02-09.
 *
 * This file is a part of Aniray
 * https://github.com/HypersonicED/aniray
 *
 * Copyright (c) 2023, Hypersonic
 * Copyright (c) 2023, Perry Naseck
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

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/log/core/record.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/trivial.hpp>
#include <modbus/modbus.h>

#include <aniray/IOInterface.hpp>
#include <aniray/IOInterfaceModbus.hpp>
#include <aniray/PeriodicThread.hpp>

namespace aniray::IOInterface::Modbus {

IOInterfaceModbus::IOInterfaceModbus(std::string tcpAddress, std::uint16_t tcpPort) {
        setupConnectionTCP(std::move(tcpAddress), tcpPort);

        // Refresh inputs to initially populate values and confirm all addresses are reachable
        // refreshInputs();
    }

IOInterfaceModbus::~IOInterfaceModbus() {
    modbus_close(mCTX);
    modbus_free(mCTX);
}

void IOInterfaceModbus::setupConnectionTCP(std::string tcpAddress, std::uint16_t tcpPort) {
    mCTX = modbus_new_tcp_pi(tcpAddress.c_str(), std::to_string(tcpPort).c_str());
    if (mCTX == nullptr) {
        throw std::runtime_error("IOInterfaceModbus: Unable to allocate libmodbus context");
    }
    if (modbus_connect(mCTX) == -1) {
        modbus_free(mCTX);
        throw std::runtime_error("IOInterfaceModbus: Connection failed: " + std::string(modbus_strerror(errno)));
    }
    BOOST_LOG_TRIVIAL(info) << "IOInterfaceModbus: Connected to "
                            << tcpAddress << ":" << tcpPort;
}

// WARNING: Not thread safe! Use setupInputDiscrete publicly!
void IOInterfaceModbus::setupInputDiscreteNoLock(const std::string &name,
                                                 std::uint8_t slaveID,
                                                 std::uint8_t functionCode,
                                                 ConfigFunctionsAddressLayout addressLayout,
                                                 std::uint16_t startAddress,
                                                 std::uint16_t numAddressedItems) {
    assignInputDiscrete(name, std::make_shared<IOInterfaceInputDiscrete>());
    // above call checks for duplicates
    mInputsDiscreteModbus[name] = {
        .name = name,
        .slaveID = slaveID,
        .functionCode = functionCode,
        .addressLayout = addressLayout,
        .startAddress = static_cast<std::uint16_t>(startAddress - 1U), // seems to always be off by one (starts at 0?)
        .numAddressedItems = numAddressedItems,
        .enableClear = false
    };
}
void IOInterfaceModbus::setupInputDiscrete(const std::string &name,
                                           std::uint8_t slaveID,
                                           std::uint8_t functionCode,
                                           ConfigFunctionsAddressLayout addressLayout,
                                           std::uint16_t startAddress,
                                           std::uint16_t numAddressedItems) {
    const std::unique_lock<std::shared_mutex> lock(mMutexInputsDiscreteModbus);
    // Use below instead of initializer so that mutex lock covers
    setupInputDiscreteNoLock(name, slaveID, functionCode, addressLayout, startAddress, numAddressedItems);
}
void IOInterfaceModbus::setupInputDiscrete(const std::string &name,
                                           std::uint8_t slaveID,
                                           std::uint8_t functionCode,
                                           ConfigFunctionsAddressLayout addressLayout,
                                           std::uint16_t startAddress,
                                           std::uint16_t numAddressedItems,
                                           std::uint8_t clearFunctionCode,
                                           std::uint16_t clearStartAddress,
                                           bool onlyClearSingleAddress,
                                           bool clearHigh) {
    const std::unique_lock<std::shared_mutex> lock(mMutexInputsDiscreteModbus);
    // Use below instead of initializer so that mutex lock covers
    setupInputDiscreteNoLock(name, slaveID, functionCode, addressLayout, startAddress, numAddressedItems);
    mInputsDiscreteModbus[name].enableClear = true;
    mInputsDiscreteModbus[name].clearFunctionCode = clearFunctionCode;
    mInputsDiscreteModbus[name].clearStartAddress = static_cast<std::uint16_t>(clearStartAddress - 1U);
    mInputsDiscreteModbus[name].onlyClearSingleAddress = onlyClearSingleAddress;
    mInputsDiscreteModbus[name].clearHigh = clearHigh;
}

void IOInterfaceModbus::refreshInputs() {
    const std::shared_lock<std::shared_mutex> inputsDiscreteLock(mMutexInputsDiscreteModbus);
    for (auto const& [name, configInputDiscrete] : mInputsDiscreteModbus) {
        updateInputDiscrete(configInputDiscrete);
    }
    // inputsDiscreteLock.unlock(); // will need this when other types of input are added
}

void IOInterfaceModbus::updateInputDiscrete(const ConfigInputDiscrete &configInputDiscrete) {
    if (modbus_set_slave(mCTX, configInputDiscrete.slaveID) == -1) {
        throw std::runtime_error("IOInterfaceModbus: Invalid slave ID: " + std::to_string(configInputDiscrete.slaveID));
    }
    std::vector<std::uint8_t> dest8(configInputDiscrete.numAddressedItems);
    std::vector<std::uint16_t> dest16(configInputDiscrete.numAddressedItems);
    int res_read = 0;
    switch (configInputDiscrete.functionCode) {
        case FUNCTION_CODE_READ_BITS:
            res_read = modbus_read_bits(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest8[0]);
            break;

        case FUNCTION_CODE_READ_INPUT_BITS:
            res_read = modbus_read_input_bits(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest8[0]);
            break;

        case FUNCTION_CODE_READ_REGISTERS:
            res_read = modbus_read_registers(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest16[0]);
            break;

        case FUNCTION_CODE_READ_INPUT_REGISTERS:
            res_read = modbus_read_input_registers(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest16[0]);
            break;

        default:
            throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input function code!");
            break;
    }
    if (res_read == -1) {
        throw std::runtime_error("IOInterfaceModbus: Error reading discrete values: " + std::string(modbus_strerror(errno)));
    }

    if (configInputDiscrete.enableClear) {
        int res_clear = 0;
        std::uint16_t numClear = configInputDiscrete.numAddressedItems;
        if (configInputDiscrete.onlyClearSingleAddress) {
            numClear = 1;
        }
        std::uint8_t clear8_value = CLEAR_BIT_VALUE_LOW;
        std::uint16_t clear16_value = CLEAR_REGISTER_VALUE_LOW;
        if (configInputDiscrete.clearHigh) {
            clear8_value = CLEAR_BIT_VALUE_HIGH;
            clear16_value = CLEAR_REGISTER_VALUE_HIGH;
        }
        std::vector<std::uint8_t> clear8(numClear, clear8_value);
        std::vector<std::uint16_t> clear16(numClear, clear16_value);
        switch (configInputDiscrete.clearFunctionCode) {
            case FUNCTION_CODE_FORCE_SINGLE_COIL:
                res_clear = modbus_write_bit(mCTX, configInputDiscrete.clearStartAddress, clear8_value);
                break;

            case FUNCTION_CODE_FORCE_MULTIPLE_COILS:
                res_clear = modbus_write_bits(mCTX, configInputDiscrete.clearStartAddress, numClear, &clear8[0]);
                break;

            case FUNCTION_CODE_PRESET_SINGLE_REGISTERS:
                res_clear = modbus_write_register(mCTX, configInputDiscrete.clearStartAddress, clear16_value);
                break;

            case FUNCTION_CODE_PRESET_MULTIPLE_REGISTERS:
                res_clear = modbus_write_registers(mCTX, configInputDiscrete.clearStartAddress, numClear, &clear16[0]);
                break;

            default:
                throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input clear function code!");
                break;
        }
        if (res_clear == -1) {
            throw std::runtime_error("IOInterfaceModbus: Error clearing discrete values: " + std::string(modbus_strerror(errno)));
        }
    }

    std::vector<bool> out;
    switch (configInputDiscrete.addressLayout) {
        case ConfigFunctionsAddressLayout::ADDRESS:
            switch (configInputDiscrete.functionCode) {
                case FUNCTION_CODE_READ_BITS:
                case FUNCTION_CODE_READ_INPUT_BITS:
                    for (std::size_t i = 0; i < configInputDiscrete.numAddressedItems; i++) {
                        out.push_back(static_cast<bool>(dest8[i]));
                    }
                    break;
                case FUNCTION_CODE_READ_REGISTERS:
                case FUNCTION_CODE_READ_INPUT_REGISTERS:
                    for (std::size_t i = 0; i < configInputDiscrete.numAddressedItems; i++) {
                        out.push_back(static_cast<bool>(dest16[i]));
                    }
                    break;
                default:
                    throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input function code!");
                    break;
            }
            break;
        case ConfigFunctionsAddressLayout::BITS_LSB:
            switch (configInputDiscrete.functionCode) {
                case FUNCTION_CODE_READ_BITS:
                case FUNCTION_CODE_READ_INPUT_BITS:
                    for (std::size_t i = 0; i < configInputDiscrete.numAddressedItems; i++) {
                        for (std::size_t bit = 0; bit < MODBUS_BITS_PER_BYTE; bit++) {
                            auto val = static_cast<std::size_t>(dest8[i] >> bit) & 1U;
                            out.push_back(static_cast<bool>(val));
                        }
                    }
                    break;
                case FUNCTION_CODE_READ_REGISTERS:
                case FUNCTION_CODE_READ_INPUT_REGISTERS:
                    for (std::size_t i = 0; i < configInputDiscrete.numAddressedItems; i++) {
                        for (std::size_t bit = 0; bit < MODBUS_BITS_PER_REGISTER; bit++) {
                            auto val = static_cast<std::size_t>(dest16[i] >> bit) & 1U;
                            out.push_back(static_cast<bool>(val));
                        }
                    }
                    break;
                default:
                    throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input function code!");
                    break;
            }
            break;
        case ConfigFunctionsAddressLayout::SPAN_2_LSB:
            // For discrete we use only the first bit, so keep it simple and skip higher bit addresses
            switch (configInputDiscrete.functionCode) {
                case FUNCTION_CODE_READ_BITS:
                case FUNCTION_CODE_READ_INPUT_BITS:
                    for (std::size_t i = 0; i < configInputDiscrete.numAddressedItems / 2; i += 2) {
                        out.push_back(static_cast<bool>(dest8[i]));
                    }
                    break;
                case FUNCTION_CODE_READ_REGISTERS:
                case FUNCTION_CODE_READ_INPUT_REGISTERS:
                    for (std::size_t i = 0; i < configInputDiscrete.numAddressedItems / 2; i += 2) {
                        out.push_back(static_cast<bool>(dest16[i]));
                    }
                    break;
                default:
                    throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input function code!");
                    break;
            }
            break;
        
        default:
            throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input address layout!");
            break;
    }
    std::shared_ptr<IOInterfaceInputDiscrete> values = getInputsDiscrete(configInputDiscrete.name);
    values->setValues(out);
}

IOInterfaceModbusThread::IOInterfaceModbusThread(std::string tcpAddress, std::uint16_t tcpPort, std::chrono::milliseconds updateRateMs)
    : IOInterfaceModbus(std::move(tcpAddress), tcpPort)
    , PeriodicThread(updateRateMs) {}

void IOInterfaceModbusThread::periodicAction() {
    refreshInputs();
}

} // namespace aniray::IOInterface::Modbus
