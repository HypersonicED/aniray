/* Geometry.hpp: Headers for geometry for Aniray systems
 *
 * Created by Perry Naseck on 2022-10-03.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
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