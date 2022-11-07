/* NodeAnimation.hpp: Headers for animating Aniray systems
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

#ifndef ANIRAY_NODEANIMATION_HPP
#define ANIRAY_NODEANIMATION_HPP

#include <cstddef>
#include <limits>
#include <typeinfo>

#include <boost/log/core/record.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/trivial.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/size.hpp>

namespace aniray {

using std::size_t;

template <typename NodeArrayT> class NodeAnimation {
public:
  using InnerNodeArrayT = NodeArrayT;

  NodeAnimation(NodeArrayT &nodeArray) : mNodeArray{nodeArray} {}

  virtual void frame([[maybe_unused]] double milliseconds) {
    if (++mFrameCount == 0) {
      BOOST_LOG_TRIVIAL(debug)
          << typeid(*this).name() << ": Frame count looping to 0.";
    }
  }

protected:
  size_t mFrameCount = std::numeric_limits<size_t>::max(); // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
  NodeArrayT &mNodeArray; // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
};

} // namespace aniray

#endif // ANIRAY_NODEANIMATION_HPP
