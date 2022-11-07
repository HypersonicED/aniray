/* Geometry.hpp: Headers for geometry for Aniray systems
 *
 * Created by Perry Naseck on 2022-10-03.
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

#ifndef ANIRAY_GEOMETRY_HPP
#define ANIRAY_GEOMETRY_HPP

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/point_xyz.hpp>
// IWYU pragma: no_forward_declare boost::geometry::cs::cartesian

namespace aniray {

using Point =
    boost::geometry::model::d3::point_xyz<double,
                                          boost::geometry::cs::cartesian>;
using MultiPoint = boost::geometry::model::multi_point<Point>;
using Box = boost::geometry::model::box<Point>;

} // namespace aniray

#endif // ANIRAY_GEOMETRY_HPP