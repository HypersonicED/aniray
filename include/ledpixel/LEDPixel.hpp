/* LEDPixel.hpp: Headers for LED Pixel systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef LEDPIXEL_LEDPIXEL_HPP
#define LEDPIXEL_LEDPIXEL_HPP

#include <ostream>

#include <boost/geometry/geometries/point_xyz.hpp>
#include <boost/geometry/io/dsv/write.hpp>

#include <ledpixel/DMXAddr.hpp>

namespace ledpixel {

using Point =
    boost::geometry::model::d3::point_xyz<double,
                                          boost::geometry::cs::cartesian>;

template <class ColorT> class LEDPixel {
public:
  using InnerColorT = ColorT;

  LEDPixel(Point coords, Point rot, DMXAddr addr, bool ignore,
           float sampleRadius)
      : mCoords{coords}, mRot{rot}, mAddr{addr}, mIgnore{ignore},
        mSampleRadius{sampleRadius}, mColor{ColorT({0, 0, 0})} {}
  LEDPixel(Point coords, Point rot)
      : mCoords{coords}, mRot{rot}, mAddr{DMXAddr()}, mIgnore{false},
        mSampleRadius{0}, mColor{ColorT({0, 0, 0})} {}

  friend auto operator<<(std::ostream &out, const LEDPixel<ColorT> &L)
      -> std::ostream & {
    out << "coords: " << boost::geometry::dsv(L.mCoords);
    out << " DMXAddr: {" << L.mAddr << "}";
    out << " ignore: " << L.mIgnore;
    out << " sampleRadius: " << L.mSampleRadius;
    return out;
  }

private:
  Point mCoords;
  Point mRot;
  DMXAddr mAddr;
  bool mIgnore;
  double mSampleRadius;
  ColorT mColor;
};

} // namespace ledpixel

#endif // LEDPIXEL_LEDPIXEL_HPP
