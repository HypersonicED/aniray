/* NodesSampler.hpp: Headers for sampling Aniray systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_NODESSAMPLER_HPP
#define ANIRAY_NODESSAMPLER_HPP

#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <vector>

#include "color/color.hpp" // IWYU pragma: keep
// IWYU pragma: no_include "color/_internal/proxy.hpp"
// IWYU pragma: no_include "color/generic/model.hpp"
// IWYU pragma: no_include "color/rgb/rgb.hpp"

#include <aniray/Geometry.hpp>

namespace aniray {

template <typename NodesT> class NodesSampler {
public:
  using InnerNodesT = NodesT;
  using NodeT = typename InnerNodesT::InnerNodeT;
  using ColorT = typename InnerNodesT::InnerNodeT::InnerColorT;

  NodesSampler(NodesT &targetLEDPixels, NodesT sourceLEDPixels)
      : mTargetLEDPixels{targetLEDPixels}, mSourceLEDPixels{sourceLEDPixels} {
    for (std::shared_ptr<NodeT> targetPixel : mTargetLEDPixels.pixels()) {
      mSamplerMap[targetPixel] = std::vector<std::shared_ptr<NodeT>>();
    }
    // NOLINT(cppcoreguidelines-init-variables)
    for (std::shared_ptr<NodeT> sourcePixel : mSourceLEDPixels.pixels()) {
      Point sourceCoords = sourcePixel->coords();
      mTargetLEDPixels.findPixelsInRadiusOfSource(
          sourceCoords,
          [&sourcePixel, this](const std::shared_ptr<NodeT> targetPixel) -> bool {
            mSamplerMap[targetPixel].push_back(sourcePixel);
            return false;
          });
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
      for (std::shared_ptr<NodeT> sourcePixel : sources) {
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
      targetPixel->color(static_cast<ColorT>(targetColor));
    }
  }

private:
  NodesT &mTargetLEDPixels;
  NodesT mSourceLEDPixels;
  std::map<std::shared_ptr<NodeT>, std::vector<std::shared_ptr<NodeT>>>
      mSamplerMap;
};

} // namespace aniray

#endif // ANIRAY_NODESSAMPLER_HPP
