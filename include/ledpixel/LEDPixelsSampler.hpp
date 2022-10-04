/* LEDPixelsSampler.hpp: Headers for LED Pixel systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef LEDPIXEL_LEDPIXELSSAMPLER_HPP
#define LEDPIXEL_LEDPIXELSSAMPLER_HPP

#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <vector>

#include "color/color.hpp" // IWYU pragma: keep
// IWYU pragma: no_include "color/_internal/proxy.hpp"
// IWYU pragma: no_include "color/generic/model.hpp"
// IWYU pragma: no_include "color/rgb/rgb.hpp"

#include <ledpixel/Geometry.hpp>

namespace ledpixel {

template <typename LEDPixelsT> class LEDPixelsSampler {
public:
  using InnerLEDPixelsT = LEDPixelsT;
  using LEDPixelT = typename InnerLEDPixelsT::InnerLEDPixelT;

  LEDPixelsSampler(LEDPixelsT &targetLEDPixels, LEDPixelsT sourceLEDPixels)
      : mTargetLEDPixels{targetLEDPixels}, mSourceLEDPixels{sourceLEDPixels} {
    for (std::shared_ptr<LEDPixelT> targetPixel : mTargetLEDPixels.pixels()) {
      mSamplerMap[targetPixel] = std::vector<std::shared_ptr<LEDPixelT>>();
    }
    // std::map<double, double> comparableDistances; //
    // NOLINT(cppcoreguidelines-init-variables)
    for (std::shared_ptr<LEDPixelT> sourcePixel : mSourceLEDPixels.pixels()) {
      Point sourceCoords = sourcePixel->coords();
      // findPixelsInRadiusOfSource(sourceCoords, [&sourcePixel,
      // this](std::shared_ptr<LEDPixelT> targetPixel) -> bool {
      findPixelsInRadiusOfSource(
          sourceCoords,
          [&sourcePixel, this](std::shared_ptr<LEDPixelT> targetPixel) -> bool {
            mSamplerMap[targetPixel].push_back(sourcePixel);
            return false;
          });
      // for ( std::shared_ptr<LEDPixelT> targetPixel: mTargetLEDPixels.pixels()
      // ) {
      //     if (targetPixel->ignore()) { continue; }
      //     Point targetCoords = targetPixel->coords();
      //     double sampleRadius = targetPixel->sampleRadius();
      //     if (std::abs(targetCoords.x() - sourceCoords.x()) > sampleRadius) {
      //     continue; } if (std::abs(targetCoords.y() - sourceCoords.y()) >
      //     sampleRadius) { continue; } if (std::abs(targetCoords.z() -
      //     sourceCoords.z()) > sampleRadius) { continue; } if
      //     (comparableDistances.count(sampleRadius) <= 0) {
      //         comparableDistances[sampleRadius] =
      //         boost::geometry::comparable_distance(
      //             Point(targetCoords.x() - sampleRadius, targetCoords.y(),
      //             targetCoords.z()), targetCoords
      //         );
      //     }
      //     double compare = 0;
      //     compare = comparableDistances[sampleRadius];
      //     double distance =
      //     boost::geometry::comparable_distance(sourceCoords, targetCoords);
      //     if (distance > compare) {
      //         continue;
      //     }
      //     mSamplerMap[targetPixel].push_back(sourcePixel);
      // }
    }
  }

  void sampleAsRGB() {
    using MixColor = color::rgb<double>;
    for (auto const &[targetPixel, sources] : mSamplerMap) {
      if (targetPixel->ignore()) {
        continue;
      }
      std::vector<double> mixR;
      std::vector<double> mixG;
      std::vector<double> mixB;
      for (std::shared_ptr<LEDPixelT> sourcePixel : sources) {
        MixColor sourceColor{};
        sourceColor = sourcePixel->color();
        mixR.push_back(sourceColor[0] * sourceColor[0]);
        mixG.push_back(sourceColor[1] * sourceColor[1]);
        mixB.push_back(sourceColor[2] * sourceColor[2]);
      }
      MixColor targetColor(
          {std::sqrt(std::accumulate(mixR.begin(), mixR.end(), 0.0) /
                     static_cast<double>(mixR.size())),
           std::sqrt(std::accumulate(mixG.begin(), mixG.end(), 0.0) /
                     static_cast<double>(mixG.size())),
           std::sqrt(std::accumulate(mixB.begin(), mixB.end(), 0.0) /
                     static_cast<double>(mixB.size()))});
      targetPixel->color(targetColor);
    }
  }

private:
  LEDPixelsT &mTargetLEDPixels;
  LEDPixelsT mSourceLEDPixels;
  std::map<std::shared_ptr<LEDPixelT>, std::vector<std::shared_ptr<LEDPixelT>>>
      mSamplerMap;
};

} // namespace ledpixel

#endif // LEDPIXEL_LEDPIXELSSAMPLER_HPP
