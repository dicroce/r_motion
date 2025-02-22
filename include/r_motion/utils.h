#ifndef __r_motion_utils_h
#define __r_motion_utils_h

#include "r_utils/r_macro.h"
#include <vector>
#include <deque>
#include <string>
#include <memory>
#include <cstdint>

namespace r_motion
{

enum r_motion_image_type
{
    R_MOTION_IMAGE_TYPE_ARGB,
    R_MOTION_IMAGE_TYPE_GRAY8,
    R_MOTION_IMAGE_TYPE_GRAY16
};

struct r_image
{
    r_motion_image_type type;
    uint16_t width;
    uint16_t height;
    std::shared_ptr<std::vector<uint8_t>> data;
};

// Structure to represent a pixel (as a point) for clustering.
struct r_point
{
    // dbscan fields
    int x;
    int y;
    int clusterId; // 0: unassigned, -1: noise, >0: cluster identifier
    bool visited;
    // hdbscan fields
    double coreDistance; // Distance to the minPts-th neighbor
};

// Edge in the mutual reachability graph.
struct r_edge
{
    int a;       // Index of point a
    int b;       // Index of point b
    double weight;
};

R_API r_image argb_to_gray8(const r_image& argb);
R_API r_image gray8_to_argb(const r_image& gray);

R_API r_image gray8_subtract(const r_image& a, const r_image& b);
R_API r_image gray8_remove(const r_image& a, const r_image& b);

R_API r_image gray8_median_filter(const r_image& input, int kernel_size = 3);
R_API r_image gray8_binarize(const r_image& input);
R_API r_image gray8_dilate(const r_image& input, int kernel_size = 3);
R_API r_image gray8_erode(const r_image& input, int kernel_size = 3);

R_API uint64_t gray8_compute_motion(const r_image& a);

R_API void write_argb_to_ppm(const std::string& filename, const r_image& image);

R_API std::vector<std::vector<r_point>> gray8_dbscan(const r_image &image, double eps, int minPts);
R_API std::vector<std::vector<r_point>> gray8_hdbscan(const r_image &image, int minPts, double clusterSelectionThreshold);

}

#endif
