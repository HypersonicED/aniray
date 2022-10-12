/* NodeArrayOutput.hpp: Headers for Aniray system outputs
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_NODEARRAYOUTPUT_HPP
#define ANIRAY_NODEARRAYOUTPUT_HPP

#include <cstdint>
#include <memory>

#include <aniray/DMXAddr.hpp>

namespace aniray {

using std::uint32_t;
using std::uint8_t;

template <typename NodeArrayT, auto ColorToOutput> class NodeArrayOutput {
public:
  using InnerNodeArrayT = NodeArrayT;

  NodeArrayOutput(NodeArrayT &nodeArray) : mNodeArray{nodeArray} {}

  auto updateAndSend() -> bool {
    using NodeT = typename InnerNodeArrayT::InnerNodeT;
    using ColorT = typename InnerNodeArrayT::InnerNodeT::InnerColorT;
    for (std::shared_ptr<NodeT> node : mNodeArray.nodes()) {
      if (node->ignore()) {
        continue;
      }
      DMXAddr addr = node->addr();
      ColorT color = node->color();
      auto output = ColorToOutput(color);
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
  virtual void setChannel(uint32_t universe, uint8_t channel, uint8_t data) {}
  virtual auto sendData() -> bool { return false; }

private:
  NodeArrayT &mNodeArray;
};

} // namespace aniray

#endif // ANIRAY_NODEARRAYOUTPUT_HPP
