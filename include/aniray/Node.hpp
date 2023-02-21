/* Node.hpp: Headers for an Aniray node
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

#ifndef ANIRAY_NODE_HPP
#define ANIRAY_NODE_HPP

#include <ios>
#include <ostream>

#include <boost/geometry/io/dsv/write.hpp>

#include <aniray/DMXAddr.hpp>
#include <aniray/Geometry.hpp>

namespace aniray {

template <class DataT> class Node {
public:
  using InnerDataT = DataT;

  Node(Point coords, Point rot, DMXAddr addr, bool ignore,
           float sampleRadius)
      : mCoords{coords}
      , mRot{rot}
      , mInGrid{false}
      , mGridIndex{PointGridIndex()}
      , mAddr{addr}
      , mIgnore{ignore}
      , mSampleRadius{sampleRadius}
      , mData{DataT()} {}
  Node(Point coords, Point rot, PointGridIndex gridIndex,
           DMXAddr addr, bool ignore, float sampleRadius)
      : mCoords{coords}
      , mRot{rot}
      , mInGrid{true}
      , mGridIndex{gridIndex}
      , mAddr{addr}
      , mIgnore{ignore}
      , mSampleRadius{sampleRadius}
      , mData{DataT()} {}
  Node(Point coords, Point rot)
      : mCoords{coords}
      , mRot{rot}
      , mInGrid{false}
      , mGridIndex{PointGridIndex()}
      , mAddr{DMXAddr()}
      , mIgnore{false}
      , mSampleRadius{0}
      , mData{DataT()} {}
  Node(Point coords, Point rot, PointGridIndex gridIndex)
      : mCoords{coords}
      , mRot{rot}
      , mInGrid{true}
      , mGridIndex{gridIndex}
      , mAddr{DMXAddr()}
      , mIgnore{false}
      , mSampleRadius{0}
      , mData{DataT()} {}

  friend auto operator<<(std::ostream &out, const Node<DataT> &L)
      -> std::ostream & {
    out << "coords: " << boost::geometry::dsv(L.mCoords);
    out << " inGrid: " << std::boolalpha << L.mInGrid << std::noboolalpha;
    out << " gridIndex: " << boost::geometry::dsv(L.mGridIndex);
    out << " DMXAddr: {" << L.mAddr << "}";
    out << " ignore: " << std::boolalpha << L.mIgnore << std::noboolalpha;
    out << " sampleRadius: " << L.mSampleRadius;
    return out;
  }

  [[nodiscard]] auto coords() const -> Point {
    return mCoords;
  }
  [[nodiscard]] auto rot() const -> Point {
    return mRot;
  }
  [[nodiscard]] auto inGrid() const -> bool {
    return mInGrid;
  }
  [[nodiscard]] auto gridIndex() const -> PointGridIndex {
    return mGridIndex;
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
  auto data() const -> DataT {
    return mData;
  }
  void data(DataT newData) {
    mData = newData;
  }

private:
  Point mCoords;
  Point mRot;
  bool mInGrid;
  PointGridIndex mGridIndex;
  DMXAddr mAddr;
  bool mIgnore;
  double mSampleRadius;
  DataT mData;
};

} // namespace aniray

#endif // ANIRAY_NODE_HPP
