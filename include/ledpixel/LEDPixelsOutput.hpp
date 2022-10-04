/* LEDPixelsOutput.hpp: Headers for LED Pixel systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef LEDPIXEL_LEDPIXELSOUTPUT_HPP
#define LEDPIXEL_LEDPIXELSOUTPUT_HPP

#include <cstdint>
#include <memory>

#include <ledpixel/DMXAddr.hpp>

namespace ledpixel {

using std::uint32_t;
using std::uint8_t;

template <typename LEDPixelsT, auto ColorToOutput> class LEDPixelsOutput {
public:
  using InnerLEDPixelsT = LEDPixelsT;

  LEDPixelsOutput(LEDPixelsT &ledPixels) : mLEDPixels{ledPixels} {}

  auto updateAndSend() -> bool {
    using LEDPixelT = typename InnerLEDPixelsT::InnerLEDPixelT;
    using ColorT = typename InnerLEDPixelsT::InnerLEDPixelT::InnerColorT;
    for (std::shared_ptr<LEDPixelT> pixel : mLEDPixels.mPixels) {
      if (pixel->mIgnore) {
        continue;
      }
      DMXAddr addr = pixel->mAddr;
      ColorT color = pixel->mColor;
      auto output = ColorToOutput(color);
      for (int i = 0; i < sizeof(output); i++) {
        setChannel(addr.mUniverse, (addr.mAddr - 1) + i, output[i]);
      }
    }
    return sendData();
  }

protected:
  virtual void setChannel(uint32_t universe, uint8_t channel, uint8_t data) {}
  virtual auto sendData() -> bool { return false; }

private:
  LEDPixelsT &mLEDPixels;
};

} // namespace ledpixel

#endif // LEDPIXEL_LEDPIXELSOUTPUT_HPP
