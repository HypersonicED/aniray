/* LEDPixelsOutputOLA.hpp: Headers for LED Pixel systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef LEDPIXEL_LEDPIXELSOUTPUTOLA_HPP
#define LEDPIXEL_LEDPIXELSOUTPUTOLA_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <boost/log/core/record.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/trivial.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/size.hpp>
#include <ola/DmxBuffer.h>
#include <ola/Logging.h>
#include <ola/client/StreamingClient.h>

#include <ledpixel/DMXAddr.hpp>
#include <ledpixel/LEDPixelsOutput.hpp>

namespace ledpixel {

using std::size_t;
using std::uint8_t;
using std::uint8_t;

template <typename LEDPixelsT, auto ColorToOutput>
class LEDPixelsOutputOLA : LEDPixelsOutput<LEDPixelsT, ColorToOutput> {
public:
  using InnerLEDPixelsT = LEDPixelsT;
  using LEDPixelsOutput<LEDPixelsT, ColorToOutput>::updateAndSend;
  using LEDPixelsOutput<LEDPixelsT, ColorToOutput>::mLEDPixels;

  LEDPixelsOutputOLA( LEDPixelsT & ledPixels)
      : LEDPixelsOutput<LEDPixelsT, ColorToOutput>::LEDPixelsOutput(ledPixels) {
    using LEDPixelT = typename InnerLEDPixelsT::InnerLEDPixelT;
    ola::InitLogging(ola::OLA_LOG_WARN, ola::OLA_LOG_STDERR);

    for (std::shared_ptr<LEDPixelT> pixel : mLEDPixels.pixels()) {
      DMXAddr addr = pixel->mAddr;
      if (mUniversesToBuffers.count(addr.mUniverse) < 1) {
        mBuffers.emplace_back();
        size_t i = mBuffers.size() - 1;
        mBuffers[i].Blackout();
        mUniversesToBuffers[addr.mUniverse] = i;
        BOOST_LOG_TRIVIAL(info) << "LEDPixelsOutputOLA: Created universe "
                                << addr.mUniverse << " buffer " << i;
      }
    }

    ola::client::StreamingClient::Options olaClientOptions =
        ola::client::StreamingClient::Options();
    olaClientOptions.auto_start = false;
    mOLAClient = std::make_unique<ola::client::StreamingClient>(
        ola::client::StreamingClient(olaClientOptions));
    if (!mOLAClient->Setup()) {
      throw std::runtime_error("LEDPixelsOutputOLA: Error setting up OLA!");
    }
    BOOST_LOG_TRIVIAL(info) << "LEDPixelsOutputOLA: Connected to OLA";
  }

private:
  std::unordered_map<uint32_t, size_t> mUniversesToBuffers;
  std::vector<ola::DmxBuffer> mBuffers;
  std::unique_ptr<ola::client::StreamingClient> mOLAClient;

  void setChannel(uint32_t universe, uint8_t channel, uint8_t data) override {
    ola::DmxBuffer &buffer = mBuffers[mUniversesToBuffers[universe]];
    buffer.SetChannel(channel, data);
  }
  auto sendData() -> bool override {
    bool res = true;
    for (auto const &[universe, i] : mUniversesToBuffers) {
      if (!mOLAClient->SendDmx(universe, mBuffers[i])) {
        res = false;
        BOOST_LOG_TRIVIAL(error)
            << "LEDPixelsOutputOLA: Send DMX failed universe " << universe;
      }
    }
    return res;
  }
};

} // namespace ledpixel

#endif // LEDPIXEL_LEDPIXELSOUTPUTOLA_HPP
