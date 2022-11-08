/* NodeArray.hpp: Headers for Aniray nodes
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

#ifndef ANIRAY_NODEARRAY_HPP
#define ANIRAY_NODEARRAY_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp> // IWYU pragma: keep
#include <boost/geometry/algorithms/envelope.hpp> // IWYU pragma: keep
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/strategies/cartesian/centroid_average.hpp> // IWYU pragma: keep
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/log/core/record.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/trivial.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/size.hpp>
#include <openssl/evp.h>
#include <openssl/ossl_typ.h>

#include "csv.h"

#include <aniray/DMXAddr.hpp>
#include <aniray/Geometry.hpp>

namespace aniray {

using std::size_t;
using std::uint8_t;

const size_t ANIRAY_NODES_FROM_CSV_NUM_COLUMNS = 11;

template <typename NodeT> class NodeArray {
public:
  using InnerNodeT = NodeT;

  // nodes and groups
  NodeArray(std::vector<NodeT> nodes,
            const std::map<std::string, std::vector<size_t>> &groups) {
    MultiPoint points;
    size_t index = 0;
    for (NodeT node : nodes) {
      mNodes.push_back(std::make_shared<NodeT>(node));
      boost::geometry::append(points, node.coords());
      for (auto const &[groupName, groupIndexes] : groups) {
        if (std::find(groupIndexes.begin(), groupIndexes.end(), index) !=
            groupIndexes.end()) {
          mGroups[groupName].push_back(mNodes.back());
        }
      }
      index++;
    }
    mCenter = boost::geometry::return_centroid<Point, MultiPoint>(points);
    mEnvelope = boost::geometry::return_envelope<Box, MultiPoint>(points);

    std::ostringstream hashInStr;
    hashInStr << *this;
    std::array<uint8_t, EVP_MAX_MD_SIZE> hash{};
    EVP_MD_CTX *m_context = EVP_MD_CTX_create();
    if (EVP_DigestInit_ex(m_context, EVP_get_digestbyname("sha512"), nullptr) ==
        0) {
      throw std::runtime_error(
          "NodeArray: Error in OpenSSL EVP_DigestInit_ex() for hash");
    }
    if (EVP_DigestUpdate(m_context, hashInStr.str().c_str(),
                         hashInStr.str().length()) == 0) {
      throw std::runtime_error(
          "NodeArray: Error in OpenSSL EVP_DigestUpdate() for hash");
    }
    unsigned int hashLength = 0;
    if (EVP_DigestFinal_ex(m_context, hash.begin(), &hashLength) == 0) {
      throw std::runtime_error(
          "NodeArray: Error in OpenSSL EVP_DigestFinal_ex() for hash");
    }
    std::ostringstream hashStream;
    for (size_t i = 0; i < hashLength; ++i) {
      hashStream << std::hex << std::setw(2) << std::setfill('0')
                 << static_cast<int>(hash.at(i));
    }
    mHash = hashStream.str();
    EVP_MD_CTX_destroy(m_context);
  }
  // nodes
  NodeArray(std::vector<NodeT> nodes)
      : NodeArray(nodes, std::map<std::string, std::vector<size_t>>{}) {}
  // nodes and groups as tuple
  NodeArray(std::tuple<std::vector<NodeT>,
                       std::map<std::string, std::vector<size_t>>>
                nodesAndGroups)
      : NodeArray(std::get<0>(nodesAndGroups), std::get<1>(nodesAndGroups)) {}
  // csv file
  NodeArray(const std::string &filename) : NodeArray{nodesFromCSV(filename)} {}
  // auto grid from target for sampler
  NodeArray(NodeArray &targetNodeArray, bool useCache, double spacing)
      : NodeArray{
            nodeGridFromTargetSpacing(targetNodeArray, useCache, spacing)} {
    if (useCache) {
      std::string cacheName = getCacheName(spacing, targetNodeArray.mHash);
      nodesToCSV(cacheName);
    }
  }

  [[nodiscard]] auto center() const -> Point {
    return mCenter;
  }

  [[nodiscard]] auto envelope() const -> Box {
    return mEnvelope;
  }

  [[nodiscard]] auto hash() const -> std::string {
    return mHash;
  }

  auto groups() -> std::map<std::string, std::vector<std::shared_ptr<NodeT>>> & {
    return mGroups;
  }

  auto nodes() -> std::vector<std::shared_ptr<NodeT>> & {
    return mNodes;
  }

  auto nodeGroupsToString(std::shared_ptr<NodeT> node,
                           std::string *groupsStr) -> size_t {
    std::ostringstream groups;
    size_t count = 0;
    for (auto const &[groupName, groupNodes] : mGroups) {
      if (std::count(groupNodes.begin(), groupNodes.end(), node) > 0) {
        if (count > 0) {
          groups << ",";
        }
        groups << groupName;
        count++;
      }
    }
    *groupsStr = groups.str();
    return count;
  }
  friend auto operator<<(std::ostream &out, NodeArray<NodeT> &L)
      -> std::ostream & {
    out << "center: " << boost::geometry::dsv(L.mCenter);
    out << " envelope: " << boost::geometry::dsv(L.mEnvelope);
    out << " nodesAndGroups: (";
    for (std::shared_ptr<NodeT> node : L.mNodes) {
      std::string nodeGroups;
      L.nodeGroupsToString(node, &nodeGroups);
      out << "{ node: {" << *node;
      out << "} groups: (" << nodeGroups << ")},";
    }
    out << ")";
    return out;
  }
  void nodesToCSV(const std::string &filename) {
    std::ofstream file(filename);
    file << "universe"
         << ",startAddr"
         << ",x"
         << ",y"
         << ",z"
         << ",xDeg"
         << ",yDeg"
         << ",zDeg"
         << ",ignore"
         << ",sampleRadius"
         << ",groups" << std::endl;
    for (std::shared_ptr<NodeT> node : mNodes) {
      std::string nodeGroups;
      size_t groupsCount = nodeGroupsToString(node, &nodeGroups);
      file << node->addr().mUniverse << "," << int(node->addr().mAddr);
      file << "," << std::to_string(node->coords().x()) << ","
           << std::to_string(node->coords().z() * -1) << ","
           << std::to_string(
                  node->coords().y()); // Real world z is up, in sim y is up
      file << "," << std::to_string(node->rot().z() * -1) << ","
           << std::to_string(node->rot().y() * -1) << ","
           << std::to_string(node->rot().x() * -1);
      file << "," << std::boolalpha << node->ignore();
      file << "," << std::to_string(node->sampleRadius());
      if (groupsCount >= 2) {
        file << ",\"";
      } else {
        file << ",";
      }
      file << nodeGroups;
      if (groupsCount >= 2) {
        file << "\"";
      }
      file << std::endl;
    }
    file.close();
  }

  template <typename onFindFunc> void findNodesInRadiusOfSource(
      Point sourceCoords, onFindFunc&& onFind) {
        findNodesInRadiusOfSource(sourceCoords, onFind, false, 0);
      }
  template <typename onFindFunc> void findNodesInRadiusOfSource(
      Point sourceCoords, onFindFunc&& onFind, bool customRadius,
      double sampleRadius) {
    static std::map<double, double> comparableDistances;
    for (std::shared_ptr<NodeT> targetNode : mNodes) {
      if (targetNode->ignore()) {
        continue;
      }
      Point targetCoords = targetNode->coords();
      if (!customRadius) {
        sampleRadius = targetNode->sampleRadius();
      }
      double sampleRadius = targetNode->sampleRadius();
      if (std::abs(targetCoords.x() - sourceCoords.x()) > sampleRadius) {
        continue;
      }
      if (std::abs(targetCoords.y() - sourceCoords.y()) > sampleRadius) {
        continue;
      }
      if (std::abs(targetCoords.z() - sourceCoords.z()) > sampleRadius) {
        continue;
      }
      if (comparableDistances.count(sampleRadius) <= 0) {
        comparableDistances[sampleRadius] =
            boost::geometry::comparable_distance(
                Point(targetCoords.x() - sampleRadius, targetCoords.y(),
                      targetCoords.z()),
                targetCoords);
      }
      double compare = 0;
      compare = comparableDistances[sampleRadius];
      double distance =
          boost::geometry::comparable_distance(sourceCoords, targetCoords);
      if (distance > compare) {
        continue;
      }
      bool breakAfterFind = onFind(targetNode);
      if (breakAfterFind) {
        break;
      }
    }
  }

private:
  std::vector<std::shared_ptr<NodeT>> mNodes;
  Point mCenter;
  Box mEnvelope;
  std::string mHash;
  std::map<std::string, std::vector<std::shared_ptr<NodeT>>> mGroups;

  static auto getCacheName(double spacing, const std::string &targetHash)
      -> std::string {
    std::ostringstream cacheName;
    cacheName << "cache_nodeGrid_spacing_" << std::to_string(spacing)
              << "_from_" << targetHash << ".csv";
    return cacheName.str();
  }
  auto nodesFromCSV(const std::string &filename) const
      -> std::tuple<std::vector<NodeT>,
                    std::map<std::string, std::vector<size_t>>> {
    std::vector<NodeT> nodes;
    io::CSVReader<ANIRAY_NODES_FROM_CSV_NUM_COLUMNS,
                  io::trim_chars<' ', '\t'>, io::double_quote_escape<',', '"'>>
        in(filename);
    in.read_header(io::ignore_extra_column, "universe", "startAddr", "x", "y",
                   "z", "xDeg", "yDeg", "zDeg", "ignore", "sampleRadius", "groups");
    uint32_t universe = 0;
    uint8_t startAddr = 0;
    double x = 0;
    double y = 0;
    double z = 0;
    double xDeg = 0;
    double yDeg = 0;
    double zDeg = 0;
    std::string ignoreStr;
    std::string groupsStr;
    bool ignore = false;
    double sampleRadius = 0;
    std::map<std::string, std::vector<size_t>> groups;
    size_t index = 0;
    // Real world z is up, in sim y is up
    while (in.read_row(universe, startAddr, x, z, y, xDeg, zDeg, yDeg,
                       ignoreStr, sampleRadius, groupsStr)) {
      boost::algorithm::trim(ignoreStr);
      boost::algorithm::to_lower(ignoreStr);
      DMXAddr addr{universe, startAddr};
      if (ignoreStr == "true") {
        ignore = true;
      } else if (ignoreStr == "false") {
        ignore = false;
      } else {
        ignore = true;
        BOOST_LOG_TRIVIAL(warning)
            << "NodeArray: Unknown ignore bool value for nodes {" << addr
            << "}, ignoring nodes";
      }
      std::stringstream groupsStream;
      groupsStream << groupsStr;
      while (groupsStream.good()) {
        std::string group;
        std::getline(groupsStream, group, ',');
        if (!group.empty()) {
          groups[group].push_back(index);
        }
      }
      nodes.push_back(NodeT(Point(x, y, z * -1),
                                    Point(zDeg * -1, yDeg * -1, xDeg * -1),
                                    addr, ignore, sampleRadius));
      index++;
    }
    return std::make_tuple(nodes, groups);
  }
  auto nodeGridFromTargetSpacing(NodeArray &targetNodeArray, bool useCache,
                                  double spacing)
      -> std::vector<NodeT> {
    if (useCache) {
      std::string cacheName = getCacheName(spacing, targetNodeArray.mHash);
      if (std::filesystem::exists(cacheName)) {
        return std::get<0>(nodesFromCSV(cacheName));
      }
    }
    Box targetEnvelope = targetNodeArray.mEnvelope;
    Point targetMin = targetEnvelope.min_corner();
    Point targetMax = targetEnvelope.max_corner();
    boost::geometry::strategy::transform::translate_transformer<double, 3, 3>
        gridStartTranslation(spacing * -1, spacing * -1, spacing * -1);
    Point gridStart;
    boost::geometry::transform(targetMin, gridStart, gridStartTranslation);
    boost::geometry::strategy::transform::translate_transformer<double, 3, 3>
        gridEndTranslation(spacing, spacing, spacing);
    Point gridEnd;
    boost::geometry::transform(targetMax, gridEnd, gridEndTranslation);
    size_t numX =
        static_cast<size_t>((gridEnd.x() - gridStart.x()) / spacing) + 1;
    size_t numY =
        static_cast<size_t>((gridEnd.y() - gridStart.y()) / spacing) + 1;
    size_t numZ =
        static_cast<size_t>((gridEnd.z() - gridStart.z()) / spacing) + 1;
    std::vector<NodeT> nodes;
    for (size_t i = 0; i < numX; i++) {
      for (size_t j = 0; j < numY; j++) {
        for (size_t k = 0; k < numZ; k++) {
          double x = static_cast<double>(i) * spacing + gridStart.x();
          double y = static_cast<double>(j) * spacing + gridStart.y();
          double z = static_cast<double>(k) * spacing + gridStart.z();
          Point newPoint(x, y, z);
          targetNodeArray.findNodesInRadiusOfSource(
              newPoint, [newPoint, &nodes]([[maybe_unused]] std::shared_ptr<NodeT> targetNode) mutable -> bool {
                nodes.push_back(NodeT(newPoint, Point(0, 0, 0)));
                return true;
              });
        }
      }
    }
    return nodes;
  }
};

} // namespace aniray

#endif // ANIRAY_NODEARRAY_HPP
