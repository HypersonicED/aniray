/* LEDPixelsOutput.hpp: Headers for Aniray system outputs
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_NODESOUTPUT_HPP
#define ANIRAY_NODESOUTPUT_HPP

#include <cstdint>
#include <memory>

#include <aniray/DMXAddr.hpp>

namespace aniray {

using std::uint32_t;
using std::uint8_t;

template <typename NodesT, auto ColorToOutput> class NodesOutput {
public:
  using InnerNodesT = NodesT;

  NodesOutput(NodesT &ledPixels) : mLEDPixels{ledPixels} {}

  auto updateAndSend() -> bool {
    using NodeT = typename InnerNodesT::InnerNodeT;
    using ColorT = typename InnerNodesT::InnerNodeT::InnerColorT;
    for (std::shared_ptr<NodeT> pixel : mLEDPixels.pixels()) {
      if (pixel->ignore()) {
        continue;
      }
      DMXAddr addr = pixel->addr();
      ColorT color = pixel->color();
      auto output = ColorToOutput(color);
      for (int i = 0; i < sizeof(output); i++) {
        setChannel(addr.mUniverse, (addr.mAddr - 1) + i, output[i]);
      }
    }
    return sendData();
  }

  auto ledPixels() const -> NodesT & {
    return mLEDPixels;
  }

protected:
  virtual void setChannel(uint32_t universe, uint8_t channel, uint8_t data) {}
  virtual auto sendData() -> bool { return false; }

private:
  NodesT &mLEDPixels;
};

} // namespace aniray

#endif // ANIRAY_NODESOUTPUT_HPP
