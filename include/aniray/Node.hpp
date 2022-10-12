/* LEDPixel.hpp: Headers for an Aniray node
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_NODE_HPP
#define ANIRAY_NODE_HPP

#include <ostream>

#include <boost/geometry/io/dsv/write.hpp>

#include <aniray/DMXAddr.hpp>
#include <aniray/Geometry.hpp>

namespace aniray {

template <class ColorT> class Node {
public:
  using InnerColorT = ColorT;

  Node(Point coords, Point rot, DMXAddr addr, bool ignore,
           float sampleRadius)
      : mCoords{coords}, mRot{rot}, mAddr{addr}, mIgnore{ignore},
        mSampleRadius{sampleRadius}, mColor{ColorT({0, 0, 0})} {}
  Node(Point coords, Point rot)
      : mCoords{coords}, mRot{rot}, mAddr{DMXAddr()}, mIgnore{false},
        mSampleRadius{0}, mColor{ColorT({0, 0, 0})} {}

  friend auto operator<<(std::ostream &out, const Node<ColorT> &L)
      -> std::ostream & {
    out << "coords: " << boost::geometry::dsv(L.mCoords);
    out << " DMXAddr: {" << L.mAddr << "}";
    out << " ignore: " << L.mIgnore;
    out << " sampleRadius: " << L.mSampleRadius;
    return out;
  }

  [[nodiscard]] auto coords() const -> Point {
    return mCoords;
  }
  [[nodiscard]] auto rot() const -> Point {
    return mRot;
  }
  [[nodiscard]] auto addr() const -> DMXAddr {
    return mAddr;
  }
  [[nodiscard]] auto ignore() const -> bool {
    return mIgnore;
  }
  [[nodiscard]] auto sampleRadius() const -> double {
    return mSampleRadius;
  }
  auto color() const -> ColorT {
    return mColor;
  }
  void color(ColorT newColor) {
    mColor = newColor;
  }

private:
  Point mCoords;
  Point mRot;
  DMXAddr mAddr;
  bool mIgnore;
  double mSampleRadius;
  ColorT mColor;
};

} // namespace aniray

#endif // ANIRAY_NODE_HPP
