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
namespace IOInterfaceModbus {

enum ConfigType {
    CONFIG_TYPE_TCP
};
struct Config {
  ConfigType type;
  std::string address;
  std::uint16_t port;
  std::vector<IOInterfaceModbusConfigFunctions> functions;
};
enum ConfigFunctionsType {
    CONFIG_FUNCTIONS_TYPE_INPUT_DISCRETE
    // CONFIG_FUNCTIONS_TYPE_INPUT_COUNTER
};
struct ConfigFunction {
  std::string name;
  ConfigFunctionsType type;
  std::uint8_t slaveID;
  std::uint8_t functionCode;
  std::uint16_t startAddress;
  std::uint16_t numItems;
//   bool clearCounterEnable;
//   std::uint8_t clearCounterFunctionCode;
//   std::uint16_t clearCounterAddress;
};

const std::uint8_t FUNCTION_CODE_READ_BITS = 1;
const std::uint8_t FUNCTION_CODE_READ_INPUT_BITS = 2;
const std::uint8_t FUNCTION_CODE_READ_REGISTERS = 3;
const std::uint8_t FUNCTION_CODE_READ_INPUT_REGISTERS = 4;

class IOInterfaceModbus : public aniray::IOInterface::IOInterface {
    public:
        IOInterfaceModbus(Config config)
            : mConfig(config) {
                switch (mConfig.type) {
                    case CONFIG_TYPE_TCP:
                        setupConnectionTCP();
                        break;
                    default:
                       throw std::runtime_error("IOInterfaceModbus: Unknown connection type!");
                       break;
                }

                for (ConfigFunction functionConfig : mConfig.functions) {
                    switch (functionConfig.type) {
                        case CONFIG_FUNCTIONS_TYPE_INPUT_DISCRETE:
                            setupInputsDiscrete(functionConfig);
                            break;
                        // case CONFIG_FUNCTIONS_TYPE_INPUT_COUNTER:
                        //     setupCounter(functionConfig);
                        //     break;
                        default:
                            throw std::runtime_error("IOInterfaceModbus: Unknown function type!");
                            break;
                    }
                }

                // Refresh inputs to initially populate values and confirm all addresses are reachable
                refreshInputs();
            }

        ~IOInterfaceModbus() {
            modbus_close(mCTX);
            modbus_free(mCTX);
        }
    private:
        Config mConfig;
        modbus_t *mCTX;

        void setupConnectionTCP() {
            mCTX = modbus_new_tcp_pi(mConfig.address.c_str(), std::to_string(mConfig.port).c_str());
            if (mCTX == NULL) {
                throw std::runtime_error("IOInterfaceModbus: Unable to allocate libmodbus context");
            } else if (modbus_connect(mCTX) == -1) {
                modbus_free(mCTX);
                throw std::runtime_error("IOInterfaceModbus: Connection failed: " + std::string(modbus_strerror(errno)));
            }
            BOOST_LOG_TRIVIAL(info) << "IOInterfaceModbus: Connected to "
                                    << mConfig.address << ":" << mConfig.port;
        }

        void setupInputsDiscrete(ConfigFunction functionConfig) {
            mInputsDiscrete.assignInputDiscrete(functionConfig.name, std::make_shared<IOInterfaceInputDiscrete>());
        }

        void refreshInputs() override {
            for (ConfigFunction functionConfig : mConfig.functions) {
                switch (functionConfig.type) {
                    case CONFIG_FUNCTIONS_TYPE_INPUT_DISCRETE:
                        updateInputsDiscrete(functionConfig);
                        break;
                    // case CONFIG_FUNCTIONS_TYPE_INPUT_COUNTER:
                    //     updateInputsCounter(functionConfig);
                    //     break;
                    default:
                        throw std::runtime_error("IOInterfaceModbus: Unknown function type!");
                        break;
                }
            }
        }

        void updateInputsDiscrete(ConfigFunction functionConfig) {
            if (modbus_set_slave(mCTX, functionConfig.slaveID) == -1) {
                throw std::runtime_error("IOInterfaceModbus: Invalid slave ID: " + std::to_string(functionConfig.slaveID));
            }
            switch (functionConfig.functionCode) {
                case FUNCTION_CODE_READ_BITS:
                    std::vector<int> dest(functionConfig.numItems);
                    modbus_read_bits(mCTX, functionConfig.startAddress, functionConfig.numItems);
                    mInputsDiscrete[functionConfig.name].setValues(&dest[0]);
                    break;

                case FUNCTION_CODE_READ_INPUT_BITS:
                    std::vector<int> dest(functionConfig.numItems);
                    modbus_read_input_bits(mCTX, functionConfig.startAddress, functionConfig.numItems);
                    mInputsDiscrete[functionConfig.name].setValues(&dest[0]);
                    break;
                
                default:
                    throw std::runtime_error("IOInterfaceModbus: Incorrect discrete input function code!");
                    break;
                
            }
        }

        // void updateInputsCounter(ConfigFunction functionConfig) {}
};

} // namespace IOInterfaceModbus
} // namespace IOInterface
} // namespace aniray

#endif // ANIRAY_INPUTINTERFACEMODBUS_HPP
