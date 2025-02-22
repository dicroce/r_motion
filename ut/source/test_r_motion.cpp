
#include "test_r_motion.h"
#include "r_motion/utils.h"
#include "r_motion/r_motion_state.h"
#include "r_utils/r_file.h"
#include "r_utils/r_string_utils.h"
#include "r_utils/r_avg.h"
#include "r_av/r_demuxer.h"
#include "r_av/r_video_decoder.h"
#include <deque>
#include <numeric>

#include "serv.h"

using namespace std;
using namespace r_utils;
using namespace r_av;
using namespace r_motion;

REGISTER_TEST_FIXTURE(test_r_motion);

std::string get_env(const string& name)
{
    std::string output;
#ifdef IS_WINDOWS
    char* s = nullptr;
    size_t len = 0;
    _dupenv_s(&s, &len, name.c_str());
    if(s)
    {
        output = string(s, len);
        free(s);
    }
#endif
#ifdef IS_LINUX
    char* env = getenv(name.c_str());
    if(env)
        output = string(env);
#endif
    return output;
}

void test_r_motion::setup()
{
    r_fs::write_file(serv_mp4, serv_mp4_len, "serv.mp4");
}

void test_r_motion::teardown()
{
    r_fs::remove_file("serv.mp4");
}

void _write_gray8(const r_image& img, const std::string& file_name)
{
    auto argb = gray8_to_argb(img);
    ppm_write_argb(file_name, argb);
}

void test_r_motion::test_basic_utils()
{
    r_demuxer demuxer("serv.mp4", true);
    auto video_stream_index = demuxer.get_video_stream_index();
    auto vsi = demuxer.get_stream_info(video_stream_index);

    auto ed = demuxer.get_extradata(video_stream_index);

    r_video_decoder decoder(vsi.codec_id);
    if(!ed.empty())
        decoder.set_extradata(ed);

    std::shared_ptr<std::vector<uint8_t>> first_frame;
    std::shared_ptr<std::vector<uint8_t>> second_frame;    
    std::shared_ptr<std::vector<uint8_t>> third_frame;

    bool got_first_key_frame = false;

    int skip = 250;
    int frame_index = 0;

    while(!first_frame || !second_frame || !third_frame)
    {
        demuxer.read_frame();
        auto fi = demuxer.get_frame_info();

        frame_index++;

        --skip;
        if(skip > 0)
            continue;

AGAIN:
        if(fi.index == video_stream_index)
        {
            r_codec_state cs = R_CODEC_STATE_INITIALIZED;

            if(!got_first_key_frame)
            {
                if(fi.key)
                {
                    got_first_key_frame = true;

                    decoder.attach_buffer(fi.data, fi.size);
                    cs = decoder.decode();
                }
            }
            else
            {
                decoder.attach_buffer(fi.data, fi.size);
                cs = decoder.decode();
            }

            if(cs == R_CODEC_STATE_AGAIN)
                goto AGAIN;

            if(cs == R_CODEC_STATE_HAS_OUTPUT || cs == R_CODEC_STATE_AGAIN_HAS_OUTPUT)
            {
                if(!first_frame)
                    first_frame = decoder.get(AV_PIX_FMT_ARGB, vsi.resolution.first, vsi.resolution.second);
                else if(!second_frame)
                    second_frame = decoder.get(AV_PIX_FMT_ARGB, vsi.resolution.first, vsi.resolution.second);
                else if(!third_frame)
                    third_frame = decoder.get(AV_PIX_FMT_ARGB, vsi.resolution.first, vsi.resolution.second);
                
                if(cs == R_CODEC_STATE_AGAIN_HAS_OUTPUT)
                    goto AGAIN;
            }
        }
    }

    RTF_ASSERT(first_frame->empty() == false);
    RTF_ASSERT(second_frame->empty() == false);
    RTF_ASSERT(third_frame->empty() == false);

    r_image first_img;
    first_img.type = R_MOTION_IMAGE_TYPE_ARGB;
    first_img.width = vsi.resolution.first;
    first_img.height = vsi.resolution.second;
    first_img.data = first_frame;

    auto first_img_bw = argb_to_gray8(first_img);

    r_image second_img;
    second_img.type = R_MOTION_IMAGE_TYPE_ARGB;
    second_img.width = vsi.resolution.first;
    second_img.height = vsi.resolution.second;
    second_img.data = second_frame;

    auto second_img_bw = argb_to_gray8(second_img);

    r_image third_img;
    third_img.type = R_MOTION_IMAGE_TYPE_ARGB;
    third_img.width = vsi.resolution.first;
    third_img.height = vsi.resolution.second;
    third_img.data = third_frame;

    auto third_img_bw = argb_to_gray8(third_img);

    // You need two frames to make a motion
    // And you need a previous motion to subtract it from the current motion... HENCE
    // you need three frames before you can start emitting motion values.

    auto diff_img = gray8_subtract(second_img_bw, first_img_bw);

    auto diff2_img = gray8_subtract(third_img_bw, second_img_bw);

    auto motion_img = gray8_remove(diff2_img, diff_img);

    auto filtered = gray8_median_filter(motion_img);
    auto binary = gray8_binarize(filtered);
    auto dilated = gray8_dilate(binary);
    auto eroded = gray8_erode(dilated);
    auto motion = gray8_compute_motion(eroded);

    RTF_ASSERT(motion > 1000);
}

void test_r_motion::test_motion_state()
{
    r_demuxer demuxer("serv.mp4", true);
    auto video_stream_index = demuxer.get_video_stream_index();
    auto vsi = demuxer.get_stream_info(video_stream_index);

    auto ed = demuxer.get_extradata(video_stream_index);

    r_video_decoder decoder(vsi.codec_id);
    if(!ed.empty())
        decoder.set_extradata(ed);

    r_motion_state ms;

    bool done_demuxing = false;

    int idx = 0;

    while(!done_demuxing)
    {
        ++idx;

        done_demuxing = !demuxer.read_frame();
        auto fi = demuxer.get_frame_info();

AGAIN:
        if(fi.index == video_stream_index && fi.key)
        {
            r_codec_state cs = R_CODEC_STATE_INITIALIZED;

            decoder.attach_buffer(fi.data, fi.size);
            cs = decoder.decode();

            if(cs == R_CODEC_STATE_AGAIN || cs == R_CODEC_STATE_HUNGRY)
                goto AGAIN;

            if(cs == R_CODEC_STATE_HAS_OUTPUT || cs == R_CODEC_STATE_AGAIN_HAS_OUTPUT)
            {
                auto frame = decoder.get(AV_PIX_FMT_ARGB, vsi.resolution.first, vsi.resolution.second);

                r_image img;
                img.type = R_MOTION_IMAGE_TYPE_ARGB;
                img.width = vsi.resolution.first;
                img.height = vsi.resolution.second;
                img.data = frame;

                auto maybe_mi = ms.process(img);

                if(!maybe_mi.is_null())
                {
                    auto mi = maybe_mi.value();
                    printf("motion=%9lu, avg_motion=%9lu, stddev=%9lu\n", mi.motion, mi.avg_motion, mi.stddev);
                    fflush(stdout);
                }
                
                if(cs == R_CODEC_STATE_AGAIN_HAS_OUTPUT)
                    goto AGAIN;
            }
        }
    }
}
