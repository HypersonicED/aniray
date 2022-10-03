/* libledpixel.hpp: Headers for LED Pixel systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef libledpixel_hpp_defined
#define libledpixel_hpp_defined

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xyz.hpp>
#include <boost/geometry/io/dsv/write.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/strategies/cartesian/centroid_average.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/log/trivial.hpp>

#ifndef LEDPIXEL_WITHOUT_OLA
#include <ola/DmxBuffer.h>
#include <ola/Logging.h>
#include <ola/client/StreamingClient.h>
#endif // LEDPIXEL_WITHOUT_OLA

#include <openssl/evp.h>
#include <openssl/sha.h>

#include "color/color.hpp"
#include "csv.h"

namespace ledpixel {
using std::size_t;
using std::uint32_t;
using std::uint8_t;

typedef boost::geometry::model::d3::point_xyz<double,
                                              boost::geometry::cs::cartesian>
    Point;
typedef boost::geometry::model::multi_point<Point> MultiPoint;
typedef boost::geometry::model::box<Point> Box;

struct DMXAddr {
  uint32_t mUniverse;
  uint8_t mAddr;
};
std::ostream &operator<<(std::ostream &out, const DMXAddr &d) {
  return out << "universe: " << d.mUniverse << " address: " << int(d.mAddr);
}

template <class ColorT> class LEDPixel {
public:
  using InnerColorT = ColorT;
  Point mCoords;
  Point mRot;
  DMXAddr mAddr;
  bool mIgnore;
  double mSampleRadius;
  ColorT mColor;

  LEDPixel(Point coords, Point rot, DMXAddr addr, bool ignore,
           float sampleRadius)
      : mCoords{coords}, mRot{rot}, mAddr{addr}, mIgnore{ignore},
        mSampleRadius{sampleRadius}, mColor{ColorT({0, 0, 0})} {}
  LEDPixel(Point coords, Point rot)
      : mCoords{coords}, mRot{rot}, mColor{ColorT({0, 0, 0})} {}

  friend std::ostream &operator<<(std::ostream &out,
                                  const LEDPixel<ColorT> &L) {
    out << "coords: " << boost::geometry::dsv(L.mCoords);
    out << " DMXAddr: {" << L.mAddr << "}";
    out << " ignore: " << L.mIgnore;
    out << " sampleRadius: " << L.mSampleRadius;
    return out;
  }
};

template <typename LEDPixelT> class LEDPixels {
public:
  using InnerLEDPixelT = LEDPixelT;
  std::vector<std::shared_ptr<LEDPixelT>> mPixels;
  Point mCenter;
  Box mEnvelope;
  std::string mHash;
  std::map<std::string, std::vector<std::shared_ptr<LEDPixelT>>> mGroups;

  LEDPixels(
      std::vector<LEDPixelT> pixels,
      std::map<std::string, std::vector<size_t>> groups) // pixels and groups
  {
    MultiPoint points;
    size_t index = 0;
    for (LEDPixelT pixel : pixels) {
      mPixels.push_back(std::make_shared<LEDPixelT>(pixel));
      boost::geometry::append(points, pixel.mCoords);
      for (auto const &[groupName, groupIndexes] : groups) {
        if (std::find(groupIndexes.begin(), groupIndexes.end(), index) !=
            groupIndexes.end()) {
          mGroups[groupName].push_back(mPixels.back());
        }
      }
      index++;
    }
    mCenter = boost::geometry::return_centroid<Point, MultiPoint>(points);
    mEnvelope = boost::geometry::return_envelope<Box, MultiPoint>(points);

    std::ostringstream hashInStr;
    hashInStr << *this;
    uint8_t hash[EVP_MAX_MD_SIZE];
    EVP_MD_CTX *m_context = EVP_MD_CTX_create();
    if (!EVP_DigestInit_ex(m_context, EVP_get_digestbyname("sha512"), NULL)) {
      throw std::runtime_error(
          "LEDPixels: Error in OpenSSL EVP_DigestInit_ex() for hash");
    }
    if (!EVP_DigestUpdate(m_context, hashInStr.str().c_str(),
                          hashInStr.str().length())) {
      throw std::runtime_error(
          "LEDPixels: Error in OpenSSL EVP_DigestUpdate() for hash");
    }
    unsigned int hashLength;
    if (!EVP_DigestFinal_ex(m_context, hash, &hashLength)) {
      throw std::runtime_error(
          "LEDPixels: Error in OpenSSL EVP_DigestFinal_ex() for hash");
    }
    std::ostringstream hashStream;
    for (size_t i = 0; i < hashLength; ++i) {
      hashStream << std::hex << std::setw(2) << std::setfill('0')
                 << (int)hash[i];
    }
    mHash = hashStream.str();
    EVP_MD_CTX_destroy(m_context);
  }
  LEDPixels(std::vector<LEDPixelT> pixels) // pixels
      : LEDPixels(pixels, std::map<std::string, std::vector<size_t>>{}) {}
  LEDPixels(std::tuple<std::vector<LEDPixelT>,
                       std::map<std::string, std::vector<size_t>>>
                pixelsAndGroups) // pixels and groups as tuple
      : LEDPixels(std::get<0>(pixelsAndGroups), std::get<1>(pixelsAndGroups)) {}
  LEDPixels(std::string filename) // csv file
      : LEDPixels{pixelsFromCSV(filename)} {}
  LEDPixels(LEDPixels &targetLEDPixels, bool useCache,
            double spacing) // auto grid from target for sampler
      : LEDPixels{
            pixelGridFromTargetSpacing(targetLEDPixels, useCache, spacing)} {
    if (useCache) {
      std::string cacheName = getCacheName(spacing, targetLEDPixels.mHash);
      pixelsToCSV(cacheName);
    }
  }

  size_t pixelGroupsToString(std::shared_ptr<LEDPixelT> pixel,
                             std::string &groupsStr) const {
    std::ostringstream groups;
    size_t count = 0;
    for (auto const &[groupName, groupPixels] : mGroups) {
      if (std::count(groupPixels.begin(), groupPixels.end(), pixel) > 0) {
        if (count > 0) {
          groups << ",";
        }
        groups << groupName;
        count++;
      }
    }
    groupsStr = groups.str();
    return count;
  }
  friend std::ostream &operator<<(std::ostream &out,
                                  const LEDPixels<LEDPixelT> &L) {
    out << "center: " << boost::geometry::dsv(L.mCenter);
    out << " envelope: " << boost::geometry::dsv(L.mEnvelope);
    out << " pixelsAndGroups: (";
    for (std::shared_ptr<LEDPixelT> pixel : L.mPixels) {
      std::string pixelGroups;
      L.pixelGroupsToString(pixel, pixelGroups);
      out << "{ pixel: {" << *pixel;
      out << "} groups: (" << pixelGroups << ")},";
    }
    out << ")";
    return out;
  }
  void pixelsToCSV(std::string filename) const {
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
         << ",groups" << std::endl;
    for (std::shared_ptr<LEDPixelT> pixel : mPixels) {
      std::string pixelGroups;
      size_t groupsCount = pixelGroupsToString(pixel, pixelGroups);
      file << pixel->mAddr.mUniverse << "," << int(pixel->mAddr.mAddr);
      file << "," << std::to_string(pixel->mCoords.x()) << ","
           << std::to_string(pixel->mCoords.z() * -1) << ","
           << std::to_string(
                  pixel->mCoords.y()); // Real world z is up, in sim y is up
      file << "," << std::to_string(pixel->mRot.z() * -1) << ","
           << std::to_string(pixel->mRot.y() * -1) << ","
           << std::to_string(pixel->mRot.x() * -1);
      file << "," << std::boolalpha << pixel->mIgnore;
      if (groupsCount >= 2) {
        file << ",\"";
      } else {
        file << ",";
      }
      file << pixelGroups;
      if (groupsCount >= 2) {
        file << "\"";
      }
      file << std::endl;
    }
    file.close();
  }

private:
  static std::string getCacheName(double spacing, std::string targetHash) {
    std::ostringstream cacheName;
    cacheName << "cache_pixelGrid_spacing_" << std::to_string(spacing)
              << "_from_" << targetHash << ".csv";
    return cacheName.str();
  }
  std::tuple<std::vector<LEDPixelT>, std::map<std::string, std::vector<size_t>>>
  pixelsFromCSV(std::string filename) const {
    std::vector<LEDPixelT> ledPixels;
    io::CSVReader<10, io::trim_chars<' ', '\t'>,
                  io::double_quote_escape<',', '"'>>
        in(filename);
    in.read_header(io::ignore_extra_column, "universe", "startAddr", "x", "y",
                   "z", "xDeg", "yDeg", "zDeg", "ignore", "groups");
    uint32_t universe;
    uint8_t startAddr;
    double x, y, z;
    double xDeg, yDeg, zDeg;
    std::string ignoreStr, groupsStr;
    bool ignore;
    std::map<std::string, std::vector<size_t>> groups;
    size_t index = 0;
    // Real world z is up, in sim y is up
    while (in.read_row(universe, startAddr, x, z, y, xDeg, zDeg, yDeg,
                       ignoreStr, groupsStr)) {
      boost::algorithm::trim(ignoreStr);
      boost::algorithm::to_lower(ignoreStr);
      DMXAddr addr{universe, startAddr};
      if (ignoreStr.compare("true") == 0) {
        ignore = true;
      } else if (ignoreStr.compare("false") == 0) {
        ignore = false;
      } else {
        ignore = true;
        BOOST_LOG_TRIVIAL(warning)
            << "LEDPixels: Unknown ignore bool value for pixel {" << addr
            << "}, ignoring pixel";
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
      ledPixels.push_back(LEDPixelT(Point(x, y, z * -1),
                                    Point(zDeg * -1, yDeg * -1, xDeg * -1),
                                    addr, ignore, 1.0f));
      index++;
    }
    return std::make_tuple(ledPixels, groups);
  }
  std::vector<LEDPixelT> pixelGridFromTargetSpacing(LEDPixels &targetLEDPixels,
                                                    bool useCache,
                                                    double spacing) const {
    if (useCache) {
      std::string cacheName = getCacheName(spacing, targetLEDPixels.mHash);
      if (std::filesystem::exists(cacheName)) {
        return std::get<0>(pixelsFromCSV(cacheName));
      }
    }
    Box targetEnvelope = targetLEDPixels.mEnvelope;
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
    size_t numX = ((gridEnd.x() - gridStart.x()) / spacing) + 1;
    size_t numY = ((gridEnd.y() - gridStart.y()) / spacing) + 1;
    size_t numZ = ((gridEnd.z() - gridStart.z()) / spacing) + 1;
    std::vector<LEDPixelT> ledPixels;
    std::map<double, double> comparableDistances;
    for (size_t i = 0; i < numX; i++) {
      for (size_t j = 0; j < numY; j++) {
        for (size_t k = 0; k < numZ; k++) {
          double x = i * spacing + gridStart.x();
          double y = j * spacing + gridStart.y();
          double z = k * spacing + gridStart.z();
          Point newPoint(x, y, z);
          for (std::shared_ptr<LEDPixelT> targetPixel :
               targetLEDPixels.mPixels) {
            if (targetPixel->mIgnore)
              continue;
            Point targetCoords = targetPixel->mCoords;
            double sampleRadius = targetPixel->mSampleRadius;
            if (std::abs(targetCoords.x() - x) > sampleRadius)
              continue;
            if (std::abs(targetCoords.y() - y) > sampleRadius)
              continue;
            if (std::abs(targetCoords.z() - z) > sampleRadius)
              continue;
            if (comparableDistances.count(sampleRadius) <= 0) {
              comparableDistances[sampleRadius] =
                  boost::geometry::comparable_distance(
                      Point(targetCoords.x() - sampleRadius, targetCoords.y(),
                            targetCoords.z()),
                      targetCoords);
            }
            double compare = comparableDistances[sampleRadius];
            double distance =
                boost::geometry::comparable_distance(newPoint, targetCoords);
            if (distance > compare) {
              continue;
            }
            ledPixels.push_back(LEDPixelT(newPoint, Point(0, 0, 0)));
            break;
          }
        }
      }
    }
    return ledPixels;
  }
};

template <typename LEDPixelsT> class LEDPixelsSampler {
public:
  using InnerLEDPixelsT = LEDPixelsT;
  typedef typename InnerLEDPixelsT::InnerLEDPixelT LEDPixelT;
  LEDPixelsT &mTargetLEDPixels;
  LEDPixelsT mSourceLEDPixels;
  std::map<std::shared_ptr<LEDPixelT>, std::vector<std::shared_ptr<LEDPixelT>>>
      mSamplerMap;

  LEDPixelsSampler(LEDPixelsT &targetLEDPixels, LEDPixelsT sourceLEDPixels,
                   bool useCache)
      : mTargetLEDPixels{targetLEDPixels}, mSourceLEDPixels{sourceLEDPixels} {
    for (std::shared_ptr<LEDPixelT> targetPixel : mTargetLEDPixels.mPixels) {
      mSamplerMap[targetPixel] = std::vector<std::shared_ptr<LEDPixelT>>();
    }
    std::map<double, double> comparableDistances;
    for (std::shared_ptr<LEDPixelT> sourcePixel : mSourceLEDPixels.mPixels) {
      for (std::shared_ptr<LEDPixelT> targetPixel : mTargetLEDPixels.mPixels) {
        if (targetPixel->mIgnore)
          continue;
        Point sourceCoords = sourcePixel->mCoords;
        Point targetCoords = targetPixel->mCoords;
        double sampleRadius = targetPixel->mSampleRadius;
        if (std::abs(targetCoords.x() - sourceCoords.x()) > sampleRadius)
          continue;
        if (std::abs(targetCoords.y() - sourceCoords.y()) > sampleRadius)
          continue;
        if (std::abs(targetCoords.z() - sourceCoords.z()) > sampleRadius)
          continue;
        if (comparableDistances.count(sampleRadius) <= 0) {
          comparableDistances[sampleRadius] =
              boost::geometry::comparable_distance(
                  Point(targetCoords.x() - sampleRadius, targetCoords.y(),
                        targetCoords.z()),
                  targetCoords);
        }
        double compare = comparableDistances[sampleRadius];
        double distance =
            boost::geometry::comparable_distance(sourceCoords, targetCoords);
        if (distance > compare) {
          continue;
        }
        mSamplerMap[targetPixel].push_back(sourcePixel);
      }
    }
  }

  void sample() {
    typedef color::rgb<double> MixColor;
    for (auto const &[targetPixel, sources] : mSamplerMap) {
      if (targetPixel->mIgnore)
        continue;
      std::vector<double> mixR;
      std::vector<double> mixG;
      std::vector<double> mixB;
      for (std::shared_ptr<LEDPixelT> sourcePixel : sources) {
        MixColor sourceColor;
        sourceColor = sourcePixel->mColor;
        mixR.push_back(sourceColor[0] * sourceColor[0]);
        mixG.push_back(sourceColor[1] * sourceColor[1]);
        mixB.push_back(sourceColor[2] * sourceColor[2]);
      }
      MixColor targetColor(
          {std::sqrt(std::accumulate(mixR.begin(), mixR.end(), 0.0) /
                     (double)mixR.size()),
           std::sqrt(std::accumulate(mixG.begin(), mixG.end(), 0.0) /
                     (double)mixG.size()),
           std::sqrt(std::accumulate(mixB.begin(), mixB.end(), 0.0) /
                     (double)mixB.size())});
      targetPixel->mColor = targetColor;
    }
  }
};

template <typename LEDPixelsT, auto ColorToOutput> class LEDPixelsOutput {
public:
  using InnerLEDPixelsT = LEDPixelsT;
  LEDPixelsT &mLEDPixels;

  LEDPixelsOutput(LEDPixelsT &ledPixels) : mLEDPixels{ledPixels} {}

  bool updateAndSend() {
    typedef typename InnerLEDPixelsT::InnerLEDPixelT LEDPixelT;
    typedef typename InnerLEDPixelsT::InnerLEDPixelT::InnerColorT ColorT;
    for (std::shared_ptr<LEDPixelT> pixel : mLEDPixels.mPixels) {
      if (pixel->mIgnore)
        continue;
      DMXAddr addr = pixel->mAddr;
      ColorT color = pixel->mColor;
      auto output = ColorToOutput(color);
      for (int i = 0; i < sizeof(output); i++) {
        setChannel(addr.mUniverse, (addr.mAddr - 1) + i, output[i]);
      }
    }
    return sendData();
  }

protected:
  virtual void setChannel(uint32_t universe, uint8_t channel, uint8_t data) {}
  virtual bool sendData() { return false; }
};

#ifndef LEDPIXEL_WITHOUT_OLA
template <typename LEDPixelsT, auto ColorToOutput>
class LEDPixelsOutputOLA : LEDPixelsOutput<LEDPixelsT, ColorToOutput> {
public:
  using InnerLEDPixelsT = LEDPixelsT;
  using LEDPixelsOutput<LEDPixelsT, ColorToOutput>::updateAndSend;
  using LEDPixelsOutput<LEDPixelsT, ColorToOutput>::mLEDPixels;

  LEDPixelsOutputOLA(LEDPixelsT &ledPixels)
      : LEDPixelsOutput<LEDPixelsT, ColorToOutput>::LEDPixelsOutput(ledPixels) {
    typedef typename InnerLEDPixelsT::InnerLEDPixelT LEDPixelT;
    ola::InitLogging(ola::OLA_LOG_WARN, ola::OLA_LOG_STDERR);

    for (std::shared_ptr<LEDPixelT> pixel : mLEDPixels.mPixels) {
      DMXAddr addr = pixel->mAddr;
      if (mUniversesToBuffers.count(addr.mUniverse) < 1) {
        mBuffers.emplace_back();
        size_t i = mBuffers.size() - 1;
        mBuffers[i].Blackout();
        mUniversesToBuffers[addr.mUniverse] = i;
        BOOST_LOG_TRIVIAL(info) << "LEDPixelsOutputOLA: Created universe "
                                << addr.mUniverse << " buffer " << i;
      }
    }

    ola::client::StreamingClient::Options olaClientOptions =
        ola::client::StreamingClient::Options();
    olaClientOptions.auto_start = false;
    mOLAClient = std::unique_ptr<ola::client::StreamingClient>(
        new ola::client::StreamingClient(olaClientOptions));
    if (!mOLAClient->Setup()) {
      throw std::runtime_error("LEDPixelsOutputOLA: Error setting up OLA!");
    }
    BOOST_LOG_TRIVIAL(info) << "LEDPixelsOutputOLA: Connected to OLA";
  }

private:
  std::unordered_map<uint32_t, size_t> mUniversesToBuffers;
  std::vector<ola::DmxBuffer> mBuffers;
  std::unique_ptr<ola::client::StreamingClient> mOLAClient;

  void setChannel(uint32_t universe, uint8_t channel, uint8_t data) override {
    ola::DmxBuffer &buffer = mBuffers[mUniversesToBuffers[universe]];
    buffer.SetChannel(channel, data);
  }
  bool sendData() override {
    bool res = true;
    for (auto const &[universe, i] : mUniversesToBuffers) {
      if (!mOLAClient->SendDmx(universe, mBuffers[i])) {
        res = false;
        BOOST_LOG_TRIVIAL(error)
            << "LEDPixelsOutputOLA: Send DMX failed universe " << universe;
      }
    }
    return res;
  }
};
#endif // LEDPIXEL_WITHOUT_OLA

template <typename LEDPixelsT> class LEDPixelAnimation {
public:
  using InnerLEDPixelsT = LEDPixelsT;
  LEDPixelsT &mLEDPixels;
  size_t mFrameCount = std::numeric_limits<size_t>::max();

  LEDPixelAnimation(LEDPixelsT &ledPixels) : mLEDPixels{ledPixels} {}

  void frame() {
    if (++mFrameCount == 0) {
      BOOST_LOG_TRIVIAL(debug)
          << typeid(*this).name() << ": Frame count looping to 0.";
    }
  }
};
} // namespace ledpixel

#endif // libledpixel_hpp_defined
