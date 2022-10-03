/* LEDPixelsAnimation.hpp: Headers for LED Pixel systems
 *
 * Created by Perry Naseck on 2022-08-24.
 *
 * Copyright (c) 2022, Hypersonic
 * All rights reserved.
 *
 * This source code is closed sourced.
 */

#ifndef LEDPIXEL_LEDPIXELANIMATION_HPP
#define LEDPIXEL_LEDPIXELANIMATION_HPP

#include <limits>

#include <boost/log/trivial.hpp>

namespace ledpixel {

template <typename LEDPixelsT>
class LEDPixelAnimation {
    public:
        using InnerLEDPixelsT = LEDPixelsT;


        LEDPixelAnimation( LEDPixelsT &ledPixels )
            : mLEDPixels{ ledPixels } {}

        void frame()
        {
            if (++mFrameCount == 0) {
                BOOST_LOG_TRIVIAL(debug) << typeid(*this).name() << ": Frame count looping to 0.";
            }
        }
    private:
        size_t mFrameCount = std::numeric_limits<size_t>::max();
        LEDPixelsT &mLEDPixels;
};

} // namespace ledpixel

#endif // LEDPIXEL_LEDPIXELANIMATION_HPP
