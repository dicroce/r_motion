#include "r_motion/r_motion_state.h"

using namespace std;
using namespace r_utils;
using namespace r_motion;

r_motion_state::r_motion_state(size_t memory) :
    _avg_motion(0, memory),
    _has_last(false),
    _last(),
    _has_last_motion(),
    _last_motion(),
    _last_width(0),
    _last_height(0),
    _bw(),
    _diff(),
    _removed(),
    _filtered(),
    _binary(),
    _dilated(),
    _eroded()
{
}

r_motion_state::~r_motion_state() noexcept
{
}

r_nullable<r_motion_info> r_motion_state::process(const r_image& argb_input)
{
    if(_resolution_change(argb_input))
    {
        _bw = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
        _normalized = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
        _diff = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
        _removed = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
        _filtered = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
        _binary = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
        _dilated = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
        _eroded = create_image(R_MOTION_IMAGE_TYPE_GRAY8, argb_input.width, argb_input.height);
    }

    r_nullable<r_motion_info> result;

    argb_to_gray8(argb_input, _bw);

    if(_has_last)
    {
        // Use the new normalized subtraction function
        gray8_subtract_normalized(_bw, _last, _diff, 0.15);

        if(_has_last_motion)
        {
            gray8_remove(_diff, _last_motion, _removed);

            gray8_median_filter(_removed, _filtered);

            gray8_binarize(_filtered, _binary);

            gray8_dilate(_binary, _dilated);

            gray8_erode(_dilated, _eroded);

            r_motion_info mi;
            mi.motion_pixels = _eroded;
            mi.motion = gray8_compute_motion(_eroded);
            mi.avg_motion = _avg_motion.update(mi.motion);
            mi.stddev = _avg_motion.standard_deviation();

            result.set_value(mi);
        }

        _has_last_motion = true;
        _last_motion = _diff;
    }

    _has_last = true;
    _last = _bw;

    return result;
}
