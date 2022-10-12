/* DMXAddr.hpp: Headers DMX addressing for Aniray systems
 *
 * Created by Perry Naseck on 2022-09-21.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_DMXADDR_HPP
#define ANIRAY_DMXADDR_HPP

#include <cstdint>
#include <ostream>

namespace aniray {

using std::uint32_t;
using std::uint8_t;

struct DMXAddr {
  uint32_t mUniverse;
  uint8_t mAddr;
};

auto operator<<(std::ostream &out, const DMXAddr &d) -> std::ostream &;

} // namespace aniray

#endif // ANIRAY_DMXADDR_HPP
