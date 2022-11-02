/* NodeAnimation.hpp: Headers for animating Aniray systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
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

  virtual void frame(double milliseconds) { // NOLINT(misc-unused-parameters)
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
