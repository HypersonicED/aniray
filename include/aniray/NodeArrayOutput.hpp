/* NodeArrayOutput.hpp: Headers for Aniray system outputs
 *
 * Created by Perry Naseck on 2022-08-24.
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

#ifndef ANIRAY_NODEARRAYOUTPUT_HPP
#define ANIRAY_NODEARRAYOUTPUT_HPP

#include <cstdint>
#include <memory>

#include <aniray/DMXAddr.hpp>

namespace aniray {

template <typename NodeArrayT, auto DataToOutput> class NodeArrayOutput {
public:
  using InnerNodeArrayT = NodeArrayT;

  NodeArrayOutput(NodeArrayT &nodeArray) : mNodeArray{nodeArray} {}

  auto updateAndSend() -> bool {
    using NodeT = typename InnerNodeArrayT::InnerNodeT;
    using DataT = typename InnerNodeArrayT::InnerNodeT::InnerDataT;
    for (std::shared_ptr<NodeT> node : mNodeArray.nodes()) {
      if (node->ignore()) {
        continue;
      }
      DMXAddr addr = node->addr();
      DataT data = node->data();
      auto output = DataToOutput(data);
      for (int i = 0; i < sizeof(output); i++) {
        setChannel(addr.mUniverse, (addr.mAddr - 1) + i, output[i]);
      }
    }
    return sendData();
  }

  auto nodeArray() const -> NodeArrayT & {
    return mNodeArray;
  }

protected:
  virtual void setChannel(std::uint32_t universe, std::uint8_t channel,
                          std::uint8_t data) {}
  virtual auto sendData() -> bool { return false; }

private:
  NodeArrayT &mNodeArray;
};

} // namespace aniray

#endif // ANIRAY_NODEARRAYOUTPUT_HPP
