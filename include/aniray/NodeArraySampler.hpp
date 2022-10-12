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

#include <map>
#include <memory>
#include <vector>

#include <aniray/Geometry.hpp>

namespace aniray {

template <typename NodeArrayT> class NodeArraySampler {
public:
  using InnerNodeArrayT = NodeArrayT;
  using NodeT = typename InnerNodeArrayT::InnerNodeT;
  using DataT = typename InnerNodeArrayT::InnerNodeT::InnerDataT;

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

  template <typename sampleFuncT> void sample(sampleFuncT&& sampleFunc) {
    for (auto const &[targetNode, sources] : mSamplerMap) {
      if (targetNode->ignore()) {
        continue;
      }
      targetNode->data(sampleFunc(targetNode, sources));
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
