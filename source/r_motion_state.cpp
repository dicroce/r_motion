#include "r_motion/r_motion_state.h"

using namespace std;
using namespace r_utils;
using namespace r_motion;

r_motion_state::r_motion_state(size_t memory) :
    _avg_motion(0, memory),
    _has_last(false),
    _last(),
    _has_last_motion(),
    _last_motion()
{
}

r_motion_state::r_motion_state(r_motion_state&& obj) :
    _avg_motion(move(obj._avg_motion)),
    _has_last(move(obj._has_last)),
    _last(move(obj._last)),
    _has_last_motion(move(obj._has_last_motion)),
    _last_motion(move(obj._last_motion))
{
}

r_motion_state::~r_motion_state() noexcept
{
}

r_motion_state& r_motion_state::operator=(r_motion_state&& obj)
{
    if(this != &obj)
    {
        _avg_motion = move(obj._avg_motion);
        _has_last = move(obj._has_last);
        _last = move(obj._last);
        _has_last_motion = move(obj._has_last_motion);
        _last_motion = move(obj._last_motion);
    }

    return *this;
}

r_nullable<r_motion_info> r_motion_state::process(const r_image& argb_input)
{
    r_nullable<r_motion_info> result;

    auto bw = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);

    argb_to_gray8(argb_input, bw);

    if(_has_last)
    {
        auto diff = create_image(R_MOTION_IMAGE_TYPE_GRAY8, bw.width, bw.height);

        gray8_subtract(bw, _last, diff);

        if(_has_last_motion)
        {
            diff = gray8_remove(diff, _last_motion);

            auto filtered = gray8_median_filter(diff);
            auto binary = gray8_binarize(filtered);
            auto dilated = gray8_dilate(binary);
            auto eroded = gray8_erode(dilated);

            r_motion_info mi;
            mi.motion_pixels = eroded;
            mi.motion = gray8_compute_motion(eroded);
            mi.avg_motion = _avg_motion.update(mi.motion);
            mi.stddev = _avg_motion.standard_deviation();

            result.set_value(mi);
        }

        _has_last_motion = true;
        _last_motion = diff;
    }

    _has_last = true;
    _last = bw;

    return result;
}