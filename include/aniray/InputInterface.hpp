/* InputInterface.hpp: Headers for inputs to Aniray systems
 *
 * Created by Perry Naseck on 2022-11-03.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef ANIRAY_INPUTINTERFACE_HPP
#define ANIRAY_INPUTINTERFACE_HPP

#include <cstdint>
#include <shared_mutex>
#include <vector>

namespace aniray {

class InputInterface {
    public:
        auto getInputs() const -> std::vector<std::uint16_t>;
    protected:
        virtual void refreshInputs() = 0;
        void setInputs(std::vector<std::uint16_t> inputs);
    private:
        mutable std::shared_mutex mInputsMutex;
        std::vector<std::uint16_t> mInputs;
};

} // namespace aniray

#endif // ANIRAY_INPUTINTERFACE_HPP
