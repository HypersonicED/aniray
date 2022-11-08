/* DMXAddr.hpp: Headers DMX addressing for Aniray systems
 *
 * Created by Perry Naseck on 2022-09-21.
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

#ifndef ANIRAY_DMXADDR_HPP
#define ANIRAY_DMXADDR_HPP

#include <cstdint>
#include <ostream>

namespace aniray {

struct DMXAddr {
  std::uint32_t mUniverse;
  std::uint8_t mAddr;
};

auto operator<<(std::ostream &out, const DMXAddr &d) -> std::ostream &;

} // namespace aniray

#endif // ANIRAY_DMXADDR_HPP
