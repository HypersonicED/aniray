/* LEDPixels.hpp: Headers for LED Pixel systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef LEDPIXEL_LEDPIXELS_HPP
#define LEDPIXEL_LEDPIXELS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <openssl/evp.h>

#include "csv/csv.h"

#include <ledpixel/LEDPixel.hpp>

namespace ledpixel {

using MultiPoint = boost::geometry::model::multi_point<Point>;
using Box = boost::geometry::model::box<Point>;
using std::uint8_t;

template <typename LEDPixelT>
class LEDPixels {
  public:
    using InnerLEDPixelT = LEDPixelT;

    // pixels and groups
    LEDPixels(std::vector<LEDPixelT> pixels, const std::map<std::string, std::vector<size_t>> &groups) // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    {
        MultiPoint points;
        size_t index = 0;
        for( LEDPixelT pixel: pixels ) {
            mPixels.push_back(std::make_shared<LEDPixelT>(pixel));
            boost::geometry::append(points, pixel.mCoords);
            for (auto const& [groupName, groupIndexes] : groups) {
                    if(std::find(groupIndexes.begin(), groupIndexes.end(), index) != groupIndexes.end()) {
                    mGroups[groupName].push_back(mPixels.back());
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
        if (EVP_DigestInit_ex(m_context, EVP_get_digestbyname("sha512"), nullptr) == 0) {
            throw std::runtime_error("LEDPixels: Error in OpenSSL EVP_DigestInit_ex() for hash");
        }
        if (EVP_DigestUpdate(m_context, hashInStr.str().c_str(), hashInStr.str().length()) == 0) {
            throw std::runtime_error("LEDPixels: Error in OpenSSL EVP_DigestUpdate() for hash");
        }
        unsigned int hashLength = 0;
        if (EVP_DigestFinal_ex(m_context, hash.begin(), &hashLength) == 0) {
            throw std::runtime_error("LEDPixels: Error in OpenSSL EVP_DigestFinal_ex() for hash");
        }
        std::ostringstream hashStream;
        for (size_t i = 0; i < hashLength; ++i) {
            hashStream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash.at(i));
        }
        mHash = hashStream.str();
        EVP_MD_CTX_destroy(m_context);
    }
    // pixels
    LEDPixels(std::vector<LEDPixelT> pixels)  // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        : LEDPixels(pixels, std::map<std::string, std::vector<size_t>>{}) {}
    // pixels and groups as tuple
    LEDPixels(std::tuple<std::vector<LEDPixelT>, std::map<std::string, std::vector<size_t>>> pixelsAndGroups)  // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        : LEDPixels(std::get<0>(pixelsAndGroups), std::get<1>(pixelsAndGroups)) {}
    // csv file
    LEDPixels(std::string filename)  // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        : LEDPixels{ pixelsFromCSV(filename) } {}
    // auto grid from target for sampler
    LEDPixels(LEDPixels &targetLEDPixels, bool useCache, double spacing)  // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        : LEDPixels{ pixelGridFromTargetSpacing(targetLEDPixels, useCache, spacing) } {
            if (useCache) {
                std::string cacheName = getCacheName(spacing, targetLEDPixels.mHash);
                pixelsToCSV(cacheName);
            }
        }

        auto pixelGroupsToString(std::shared_ptr<LEDPixelT> pixel, std::string &groupsStr) const -> size_t {
            std::ostringstream groups;
            size_t count = 0;
            for (auto const& [groupName, groupPixels] : mGroups) {
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
        friend auto operator << (std::ostream &out, const LEDPixels<LEDPixelT> &L) -> std::ostream & {
            out << "center: " << boost::geometry::dsv(L.mCenter);
            out << " envelope: " << boost::geometry::dsv(L.mEnvelope);
            out << " pixelsAndGroups: (";
            for( std::shared_ptr<LEDPixelT> pixel: L.mPixels ) {
                std::string pixelGroups;
                L.pixelGroupsToString(pixel, pixelGroups);
                out << "{ pixel: {" << *pixel;
                out << "} groups: (" << pixelGroups << ")},";
            }
            out << ")";
            return out;
        }
        void pixelsToCSV(std::string filename) const
        {
            std::ofstream file(filename);
            file << "universe" << ",startAddr" << ",x" << ",y" << ",z" << ",xDeg" << ",yDeg" << ",zDeg" << ",ignore" << ",groups" << std::endl;
            for( std::shared_ptr<LEDPixelT> pixel: mPixels ) {
                std::string pixelGroups;
                size_t groupsCount = pixelGroupsToString(pixel, pixelGroups);
                file << pixel->mAddr.mUniverse << "," << int(pixel->mAddr.mAddr);
                file << "," << std::to_string(pixel->mCoords.x()) << "," << std::to_string(pixel->mCoords.z()*-1) << "," << std::to_string(pixel->mCoords.y()); // Real world z is up, in sim y is up
                file << "," << std::to_string(pixel->mRot.z()*-1) << "," << std::to_string(pixel->mRot.y()*-1) << "," << std::to_string(pixel->mRot.x()*-1);
                file << "," << std::boolalpha << pixel->mIgnore;
                if (groupsCount >= 2) { file << ",\""; }
                else { file << ","; }
                file << pixelGroups;
                if (groupsCount >= 2) { file << "\""; }
                file << std::endl;
            }
            file.close();
        }
    private:
        std::vector<std::shared_ptr<LEDPixelT>> mPixels;
        Point mCenter;
        Box mEnvelope;
        std::string mHash;
        std::map<std::string, std::vector<std::shared_ptr<LEDPixelT>>> mGroups;

        static auto getCacheName(double spacing, const std::string &targetHash) -> std::string {
            std::ostringstream cacheName;
            cacheName << "cache_pixelGrid_spacing_" << std::to_string(spacing) << "_from_" << targetHash << ".csv";
            return cacheName.str();
        }
        auto pixelsFromCSV(const std::string &filename) const -> std::tuple<std::vector<LEDPixelT>, std::map<std::string, std::vector<size_t>>> {
            std::vector<LEDPixelT> ledPixels;
            io::CSVReader<
                10,
                io::trim_chars<' ', '\t'>,
                io::double_quote_escape<',', '"'>
            >in(filename);
            in.read_header(io::ignore_extra_column, "universe", "startAddr", "x", "y", "z", "xDeg", "yDeg", "zDeg", "ignore", "groups");
            uint32_t universe;
            uint8_t startAddr;
            double x, y, z;
            double xDeg, yDeg, zDeg;
            std::string ignoreStr, groupsStr;
            bool ignore;
            std::map<std::string, std::vector<size_t>> groups;
            size_t index = 0;
            // Real world z is up, in sim y is up
            while (in.read_row(universe, startAddr, x, z, y, xDeg, zDeg, yDeg, ignoreStr, groupsStr)) {
                boost::algorithm::trim(ignoreStr);
                boost::algorithm::to_lower(ignoreStr);
                DMXAddr addr{universe, startAddr};
                if (ignoreStr.compare("true") == 0) {
                    ignore = true;
                } else if (ignoreStr.compare("false") == 0) {
                    ignore = false;
                } else {
                    ignore = true;
                    BOOST_LOG_TRIVIAL(warning) << "LEDPixels: Unknown ignore bool value for pixel {" << addr << "}, ignoring pixel";
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
                ledPixels.push_back(LEDPixelT(
                    Point(x, y, z*-1),
                    Point(zDeg*-1, yDeg*-1, xDeg*-1),
                    addr,
                    ignore,
                    1.0f
                ));
                index++;
            }
            return std::make_tuple(ledPixels, groups);
        }
        std::vector<LEDPixelT> pixelGridFromTargetSpacing(LEDPixels &targetLEDPixels, bool useCache, double spacing) const
        {
            if (useCache) {
                std::string cacheName = getCacheName(spacing, targetLEDPixels.mHash);
                if (std::filesystem::exists(cacheName)) {
                    return std::get<0>(pixelsFromCSV(cacheName));
                }
            }
            Box targetEnvelope = targetLEDPixels.mEnvelope;
            Point targetMin = targetEnvelope.min_corner();
            Point targetMax = targetEnvelope.max_corner();
            boost::geometry::strategy::transform::translate_transformer<double, 3, 3> gridStartTranslation(spacing * -1, spacing * -1, spacing * -1);
            Point gridStart;
            boost::geometry::transform(targetMin, gridStart, gridStartTranslation);
            boost::geometry::strategy::transform::translate_transformer<double, 3, 3> gridEndTranslation(spacing, spacing, spacing);
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
                        for ( std::shared_ptr<LEDPixelT> targetPixel: targetLEDPixels.mPixels ) {
                            if (targetPixel->mIgnore) continue;
                            Point targetCoords = targetPixel->mCoords;
                            double sampleRadius = targetPixel->mSampleRadius;
                            if (std::abs(targetCoords.x() - x) > sampleRadius) continue;
                            if (std::abs(targetCoords.y() - y) > sampleRadius) continue;
                            if (std::abs(targetCoords.z() - z) > sampleRadius) continue;
                            if (comparableDistances.count(sampleRadius) <= 0) {
                                comparableDistances[sampleRadius] = boost::geometry::comparable_distance(
                                    Point(targetCoords.x() - sampleRadius, targetCoords.y(), targetCoords.z()),
                                    targetCoords
                                );
                            }
                            double compare = comparableDistances[sampleRadius];
                            double distance = boost::geometry::comparable_distance(newPoint, targetCoords);
                            if (distance > compare) {
                                continue;
                            }
                            ledPixels.push_back(LEDPixelT(
                                newPoint,
                                Point(0,0,0)
                            ));
                            break;
                        }
                    }
                }
            }
            return ledPixels;
        }
};

} // namespace ledpixel

#endif // LEDPIXEL_LEDPIXELS_HPP
