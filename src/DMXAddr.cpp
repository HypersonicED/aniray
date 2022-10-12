/* DMXAddr.cpp: DMX addressing for Aniray systems
 *
 * Created by Perry Naseck on 2022-09-22.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#include <aniray/DMXAddr.hpp>

namespace aniray {

auto operator<<(std::ostream &out, const DMXAddr &d) -> std::ostream & {
  return out << "universe: " << d.mUniverse << " address: " << int(d.mAddr);
}

} // namespace aniray
