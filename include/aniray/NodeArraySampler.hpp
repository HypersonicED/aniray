/* NodeArraySampler.hpp: Headers for sampling Aniray systems
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
