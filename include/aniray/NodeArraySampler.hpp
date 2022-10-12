/* NodeArraySampler.hpp: Headers for sampling Aniray systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_NODEARRAYSAMPLER_HPP
#define ANIRAY_NODEARRAYSAMPLER_HPP

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

template <typename NodeArrayT> class NodeArraySampler {
public:
  using InnerNodeArrayT = NodeArrayT;
  using NodeT = typename InnerNodeArrayT::InnerNodeT;
  using ColorT = typename InnerNodeArrayT::InnerNodeT::InnerColorT;

  NodeArraySampler(NodeArrayT &targetNodeArray, NodeArrayT sourceNodeArray)
      : mTargetNodeArray{targetNodeArray}, mSourceNodeArray{sourceNodeArray} {
    for (std::shared_ptr<NodeT> targetNode : mTargetNodeArray.nodes()) {
      mSamplerMap[targetNode] = std::vector<std::shared_ptr<NodeT>>();
    }
    // NOLINT(cppcoreguidelines-init-variables)
    for (std::shared_ptr<NodeT> sourceNode : mSourceNodeArray.nodes()) {
      Point sourceCoords = sourceNode->coords();
      mTargetNodeArray.findNodesInRadiusOfSource(
          sourceCoords,
          [&sourceNode, this](const std::shared_ptr<NodeT> targetNode) -> bool {
            mSamplerMap[targetNode].push_back(sourceNode);
            return false;
          });
    }
  }

  void sampleAsRGB() {
    using MixColor = color::rgb<double>;
    for (auto const &[targetNode, sources] : mSamplerMap) {
      if (targetNode->ignore()) {
        continue;
      }
      std::vector<double> mixR;
      std::vector<double> mixG;
      std::vector<double> mixB;
      for (std::shared_ptr<NodeT> sourceNode : sources) {
        MixColor sourceColor{};
        sourceColor = sourceNode->color();
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
      targetNode->color(static_cast<ColorT>(targetColor));
    }
  }

private:
  NodeArrayT &mTargetNodeArray;
  NodeArrayT mSourceNodeArray;
  std::map<std::shared_ptr<NodeT>, std::vector<std::shared_ptr<NodeT>>>
      mSamplerMap;
};

} // namespace aniray

#endif // ANIRAY_NODEARRAYSAMPLER_HPP
