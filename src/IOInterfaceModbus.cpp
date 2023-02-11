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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/log/core/record.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/trivial.hpp>
#include <modbus/modbus.h>

#include <aniray/IOInterfaceModbus.hpp>

namespace aniray {
namespace IOInterface {
namespace Modbus {

IOInterfaceModbus::IOInterfaceModbus(std::string tcpAddress, std::uint16_t tcpPort) {
        setupConnectionTCP(tcpAddress, tcpPort);

        // Refresh inputs to initially populate values and confirm all addresses are reachable
        // refreshInputs();
    }

IOInterfaceModbus::~IOInterfaceModbus() {
    modbus_close(mCTX);
    modbus_free(mCTX);
}

void IOInterfaceModbus::setupConnectionTCP(std::string tcpAddress, std::uint16_t tcpPort) {
    mCTX = modbus_new_tcp_pi(tcpAddress.c_str(), std::to_string(tcpPort).c_str());
    if (mCTX == NULL) {
        throw std::runtime_error("IOInterfaceModbus: Unable to allocate libmodbus context");
    } else if (modbus_connect(mCTX) == -1) {
        modbus_free(mCTX);
        throw std::runtime_error("IOInterfaceModbus: Connection failed: " + std::string(modbus_strerror(errno)));
    }
    BOOST_LOG_TRIVIAL(info) << "IOInterfaceModbus: Connected to "
                            << tcpAddress << ":" << tcpPort;
}

void IOInterfaceModbus::setupInputDiscrete(std::string name,
                                           std::uint8_t slaveID,
                                           std::uint8_t functionCode,
                                           ConfigFunctionsAddressLayout addressLayout,
                                           std::uint16_t startAddress,
                                           std::uint16_t numAddressedItems,
                                           bool enableClear,
                                           std::uint8_t clearFunctionCode,
                                           bool clearSingleAddress,
                                           std::uint16_t clearAddress) {
    assignInputDiscrete(name, std::make_shared<IOInterfaceInputDiscrete>());
    // above call checks for duplicates
    const std::unique_lock<std::shared_mutex> lock(mMutexInputsDiscreteModbus);
    mInputsDiscreteModbus[name] = {
        name: name,
        slaveID: slaveID,
        functionCode: functionCode,
        addressLayout: addressLayout,
        startAddress: static_cast<std::uint16_t>(startAddress - 1U), // seems to always be off by one (starts at 0?)
        numAddressedItems: numAddressedItems,
        enableClear: enableClear,
        clearFunctionCode: clearFunctionCode,
        clearSingleAddress: clearSingleAddress,
        clearAddress: clearAddress
    };
}

void IOInterfaceModbus::refreshInputs() {
    const std::shared_lock<std::shared_mutex> inputsDiscreteLock(mMutexInputsDiscreteModbus);
    for (auto const& [name, configInputDiscrete] : mInputsDiscreteModbus) {
        updateInputDiscrete(configInputDiscrete);
    }
    // inputsDiscreteLock.unlock(); // will need this when other types of input are added
}

void IOInterfaceModbus::updateInputDiscrete(ConfigInputDiscrete configInputDiscrete) {
    if (modbus_set_slave(mCTX, configInputDiscrete.slaveID) == -1) {
        throw std::runtime_error("IOInterfaceModbus: Invalid slave ID: " + std::to_string(configInputDiscrete.slaveID));
    }
    std::vector<std::uint8_t> dest8(configInputDiscrete.numAddressedItems);
    std::vector<std::uint16_t> dest16(configInputDiscrete.numAddressedItems);
    switch (configInputDiscrete.functionCode) {
        case FUNCTION_CODE_READ_BITS:
            modbus_read_bits(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest8[0]);
            break;

        case FUNCTION_CODE_READ_INPUT_BITS:
            modbus_read_input_bits(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest8[0]);
            break;

        case FUNCTION_CODE_READ_REGISTERS:
            modbus_read_registers(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest16[0]);
            break;

        case FUNCTION_CODE_READ_INPUT_REGISTERS:
            modbus_read_input_registers(mCTX, configInputDiscrete.startAddress, configInputDiscrete.numAddressedItems, &dest16[0]);
            break;
        
        default:
            throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input function code!");
            break;
        
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
                        for (std::size_t i = 0; i < 8; i++) {
                            auto val = (dest8[i] >> i) & 1;
                            out.push_back(static_cast<bool>(val));
                        }
                    }
                    break;
                case FUNCTION_CODE_READ_REGISTERS:
                case FUNCTION_CODE_READ_INPUT_REGISTERS:
                    for (std::size_t i = 0; i < configInputDiscrete.numAddressedItems; i++) {
                        for (std::size_t i = 0; i < 16; i++) {
                            auto val = (dest16[i] >> i) & 1;
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

} // namespace Modbus
} // namespace IOInterface
} // namespace aniray
