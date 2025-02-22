#include "r_motion/utils.h"
#include "r_utils/r_file.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>

using namespace std;
using namespace r_utils;
using namespace r_motion;

r_image r_motion::argb_to_gray8(const r_image& argb)
{
    const uint8_t* src = argb.data->data();
    auto result = make_shared<vector<uint8_t>>(argb.width * argb.height);
    uint8_t* dst = result->data();

    for(uint16_t y = 0; y < argb.height; ++y)
    {
        for(uint16_t x = 0; x < argb.width; ++x)
        {
            ++src;
            uint32_t sum = *src;
            ++src;
            sum += *src;
            ++src;
            sum += *src;
            ++src;

            *dst = sum / 3;

            ++dst;
        }
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_GRAY8;
    output.width = argb.width;
    output.height = argb.height;
    output.data = result;
    return output;
}

r_image r_motion::gray8_to_argb(const r_image& gray)
{
    const uint8_t* src = gray.data->data();
    auto result = make_shared<vector<uint8_t>>((gray.width*4) * gray.height);
    uint8_t* dst = result->data();

    for(uint16_t y = 0; y < gray.height; ++y)
    {
        for(uint16_t x = 0; x < gray.width; ++x)
        {
            *dst = 255;
            ++dst;
            *dst = *src;
            ++dst;
            *dst = *src;
            ++dst;
            *dst = *src;
            ++dst;

            ++src;
        }
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_ARGB;
    output.width = gray.width;
    output.height = gray.height;
    output.data = result;
    return output;
}

r_image r_motion::gray8_subtract(const r_image& a, const r_image& b)
{ 
    const uint8_t* src_a = a.data->data();
    const uint8_t* src_b = b.data->data();

    auto result = make_shared<vector<uint8_t>>(a.width * a.height);
    uint8_t* dst = result->data();

    for(uint16_t h = 0; h < a.height; ++h)
    {
        for(uint16_t w = 0; w < a.width; ++w)
        {
            uint8_t diff = std::abs(*src_a - *src_b);
            *dst = (diff>32)?diff:0;
            ++src_a;
            ++src_b;
            ++dst;
        }
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_GRAY8;
    output.width = a.width;
    output.height = a.height;
    output.data = result;

    return output;
}

r_image r_motion::gray8_remove(const r_image& a, const r_image& b)
{
    auto output_buffer = make_shared<vector<uint8_t>>(a.width * a.height);

    for(uint16_t h = 0; h < a.height; ++h)
    {
        for(uint16_t w = 0; w < a.width; ++w)
        {
            auto a_ofs = (h*a.width) + w;
            auto a_val = a.data->data()[a_ofs];

            if(a_val > 0)
                output_buffer->data()[a_ofs] = (b.data->data()[(h*b.width) + w] == 0)?a_val:0;
        }
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_GRAY8;
    output.width = a.width;
    output.height = a.height;
    output.data = output_buffer;

    return output;
}

r_image r_motion::gray8_median_filter(const r_image& input, int kernel_size)
{
    if (input.type != R_MOTION_IMAGE_TYPE_GRAY8)
        R_THROW(("gray8_median_filter() supports only GRAY8 images"));

    if (kernel_size < 1 || (kernel_size % 2) == 0)
        R_THROW(("Kernel size must be positive and odd"));

    int half = kernel_size / 2;
    uint16_t width = input.width;
    uint16_t height = input.height;
    const uint8_t* src = input.data->data();

    auto result = make_shared<vector<uint8_t>>(width * height);

    // Temporary container to store neighborhood pixel values.
    vector<uint8_t> neighborhood;
    neighborhood.reserve(kernel_size * kernel_size);

    for (uint16_t y = 0; y < height; ++y)
    {
        for (uint16_t x = 0; x < width; ++x)
        {
            // If we're too close to the border, copy the pixel as is.
            if (x < half || y < half || x >= width - half || y >= height - half)
            {
                (*result)[y * width + x] = src[y * width + x];
            }
            else
            {
                neighborhood.clear();
                // Collect pixels within the kernel window.
                for (int dy = -half; dy <= half; ++dy)
                {
                    for (int dx = -half; dx <= half; ++dx)
                    {
                        uint8_t pixel = src[(y + dy) * width + (x + dx)];
                        neighborhood.push_back(pixel);
                    }
                }
                // Sort and pick the median value.
                sort(neighborhood.begin(), neighborhood.end());
                uint8_t median = neighborhood[neighborhood.size() / 2];
                (*result)[y * width + x] = median;
            }
        }
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_GRAY8;
    output.width = width;
    output.height = height;
    output.data = result;
    return output;
}

r_image r_motion::gray8_binarize(const r_image& input)
{
    if(input.type != R_MOTION_IMAGE_TYPE_GRAY8)
        R_THROW(("binarize() supports only GRAY8 images"));

    auto result = make_shared<vector<uint8_t>>(input.width * input.height);
    const uint8_t* src = input.data->data();

    for (uint32_t i = 0; i < input.width * input.height; ++i)
    {
        // Set any non-zero pixel to 255, making it binary.
        (*result)[i] = (src[i] > 0) ? 255 : 0;
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_GRAY8;
    output.width = input.width;
    output.height = input.height;
    output.data = result;
    return output;
}

// Dilation for a GRAY8 image using a square kernel of size kernel_size x kernel_size.
// The kernel size must be odd. For each pixel (except near boundaries),
// we set the output to 255 if any pixel in the neighborhood is 255.
r_image r_motion::gray8_dilate(const r_image& input, int kernel_size)
{
    if(input.type != R_MOTION_IMAGE_TYPE_GRAY8)
        R_THROW(("dilate() supports only GRAY8 images"));

    if(kernel_size < 1 || (kernel_size % 2) == 0)
        R_THROW(("Kernel size must be positive and odd"));

    int half = kernel_size / 2;
    uint16_t width = input.width;
    uint16_t height = input.height;
    const uint8_t* src = input.data->data();

    auto result = make_shared<vector<uint8_t>>(width * height);

    for (uint16_t y = 0; y < height; ++y)
    {
        for (uint16_t x = 0; x < width; ++x)
        {
            // If we're too close to the border, simply copy the pixel.
            if (x < half || y < half || x >= width - half || y >= height - half)
            {
                (*result)[y * width + x] = src[y * width + x];
            }
            else
            {
                uint8_t max_val = 0;
                // Iterate over the neighborhood
                for (int dy = -half; dy <= half; ++dy)
                {
                    for (int dx = -half; dx <= half; ++dx)
                    {
                        uint8_t pixel = src[(y + dy) * width + (x + dx)];
                        if (pixel > max_val)
                        {
                            max_val = pixel;
                            // Early exit if we find an "on" pixel.
                            if(max_val == 255)
                                break;
                        }
                    }
                    if(max_val == 255)
                        break;
                }
                (*result)[y * width + x] = max_val;
            }
        }
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_GRAY8;
    output.width = width;
    output.height = height;
    output.data = result;
    return output;
}

// Erosion for a GRAY8 image using a square kernel of size kernel_size x kernel_size.
// The kernel size must be odd. For each pixel (except near boundaries),
// we set the output to 255 only if every pixel in the neighborhood is 255; otherwise 0.
r_image r_motion::gray8_erode(const r_image& input, int kernel_size)
{
    if(input.type != R_MOTION_IMAGE_TYPE_GRAY8)
        R_THROW(("erode() supports only GRAY8 images"));

    if(kernel_size < 1 || (kernel_size % 2) == 0)
        R_THROW(("Kernel size must be positive and odd"));

    int half = kernel_size / 2;
    uint16_t width = input.width;
    uint16_t height = input.height;
    const uint8_t* src = input.data->data();

    auto result = make_shared<vector<uint8_t>>(width * height);

    for (uint16_t y = 0; y < height; ++y)
    {
        for (uint16_t x = 0; x < width; ++x)
        {
            // If we're too close to the border, simply copy the pixel.
            if (x < half || y < half || x >= width - half || y >= height - half)
            {
                (*result)[y * width + x] = src[y * width + x];
            }
            else
            {
                uint8_t min_val = 255;
                // Iterate over the neighborhood
                for (int dy = -half; dy <= half; ++dy)
                {
                    for (int dx = -half; dx <= half; ++dx)
                    {
                        uint8_t pixel = src[(y + dy) * width + (x + dx)];
                        if (pixel < min_val)
                        {
                            min_val = pixel;
                            // Early exit if we find an "off" pixel.
                            if(min_val == 0)
                                break;
                        }
                    }
                    if(min_val == 0)
                        break;
                }
                (*result)[y * width + x] = min_val;
            }
        }
    }

    r_image output;
    output.type = R_MOTION_IMAGE_TYPE_GRAY8;
    output.width = width;
    output.height = height;
    output.data = result;
    return output;
}

uint64_t r_motion::gray8_compute_motion(const r_image& a)
{
    auto a_p = a.data->data();
    uint64_t sum = 0;

    for(uint16_t h = 0; h < a.height; ++h)
    {
        for(uint16_t w = 0; w < a.width; ++w)
        {
            sum += *a_p;
            ++a_p;
        }
    }

    return sum;
}

void r_motion::write_argb_to_ppm(const std::string& filename, const r_image& image)
{
    auto outFile = r_file::open(filename, "w+b");

    fprintf(outFile, "P6 %d %d 255\n", image.width, image.height);

    const uint8_t* src = image.data->data();

    for(uint16_t y = 0; y < image.height; ++y)
    {
        for(uint16_t x = 0; x < image.width; ++x)
        {
            ++src;

            fwrite(src, 1, 1, outFile);
            ++src;
            fwrite(src, 1, 1, outFile);
            ++src;
            fwrite(src, 1, 1, outFile);
            ++src;
        }
    }

    outFile.close();
}

// Euclidean distance between two points
static double _distance(const r_point &a, const r_point &b)
{
    int dx = a.x - b.x;
    int dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Returns indices of points within eps of point at index 'idx'
static std::vector<int> _regionQuery(const std::vector<r_point> &points, int idx, double eps)
{
    std::vector<int> neighbors;
    const r_point &p = points[idx];
    for (int i = 0; i < points.size(); ++i)
    {
        if (_distance(p, points[i]) <= eps)
            neighbors.push_back(i);
    }
    return neighbors;
}

// DBSCAN clustering on an r_image (assumed to be R_MOTION_IMAGE_TYPE_GRAY8)
// clusters: output vector where each element is a cluster (a vector of Points).
std::vector<std::vector<r_point>> r_motion::gray8_dbscan(const r_image &image, double eps, int minPts)
{
    if(image.type != R_MOTION_IMAGE_TYPE_GRAY8)
        R_THROW(("gray8_dbscan() supports only GRAY8 images"));

    // Gather points: only nonzero pixels are considered (threshold = 0)
    std::vector<r_point> points;
    // Reserve an estimated size (optional optimization)
    points.reserve(image.width * image.height / 4);
    for (int y = 0; y < image.height; ++y)
    {
        for (int x = 0; x < image.width; ++x)
        {
            // Calculate the index in the 1D data buffer
            uint8_t value = (*image.data)[y * image.width + x];
            if (value > 0)
                points.push_back({x, y, 0, false});
        }
    }

    int clusterId = 1;
    // Process each point
    for (int i = 0; i < points.size(); ++i)
    {
        if (points[i].visited)
            continue;
        points[i].visited = true;
        std::vector<int> neighborIndices = _regionQuery(points, i, eps);
        if (neighborIndices.size() < minPts)
            points[i].clusterId = -1; // Mark as noise
        else
        {
            // Start a new cluster
            points[i].clusterId = clusterId;
            // Seeds list to expand the cluster
            std::vector<int> seeds = neighborIndices;
            // Expand the cluster
            for (int j = 0; j < seeds.size(); ++j)
            {
                int currIdx = seeds[j];
                if (!points[currIdx].visited)
                {
                    points[currIdx].visited = true;
                    std::vector<int> result = _regionQuery(points, currIdx, eps);
                    if (result.size() >= minPts)
                        seeds.insert(seeds.end(), result.begin(), result.end()); // Append new neighbors to the seeds list
                }
                // If not yet assigned to a cluster, add it to the current cluster
                if (points[currIdx].clusterId == 0)
                    points[currIdx].clusterId = clusterId;
            }
            // Move on to next cluster id for subsequent clusters
            clusterId++;
        }
    }

    // Organize points into clusters (ignoring noise if desired)
    std::unordered_map<int, std::vector<r_point>> clusterMap;
    for (const auto &p : points)
        clusterMap[p.clusterId].push_back(p);

    std::vector<std::vector<r_point>> clusters;
    // Transfer clusters to the output vector (skipping noise, clusterId -1)
    for (auto &entry : clusterMap)
    {
        if (entry.first != -1)
            clusters.push_back(entry.second);
    }

    return clusters;
}

// Compute core distance for each point using the minPts-th nearest neighbor.
// The core distance for a point is the distance to its minPts-th closest neighbor.
void _computeCoreDistances(std::vector<r_point> &points, int minPts)
{
    int n = points.size();
    for (int i = 0; i < n; ++i)
    {
        std::vector<double> distances;
        distances.reserve(n - 1);
        for (int j = 0; j < n; ++j)
        {
            if (i == j)
                continue;
            distances.push_back(_distance(points[i], points[j]));
        }
        std::sort(distances.begin(), distances.end());
        if (distances.size() >= static_cast<size_t>(minPts))
            points[i].coreDistance = distances[minPts - 1]; // zero-indexed: minPts-th nearest is at index minPts-1
        else
            points[i].coreDistance = std::numeric_limits<double>::infinity();
    }
}

// Mutual reachability distance between two points
static double _mutualReachabilityDistance(const r_point &a, const r_point &b)
{
    double d = _distance(a, b);
    return std::max({a.coreDistance, b.coreDistance, d});
}

// Union-find data structure for Kruskal's algorithm.
struct union_find
{
    std::vector<int> parent;
    union_find(int n) : parent(n)
    {
        for (int i = 0; i < n; ++i)
            parent[i] = i;
    }
    int find(int i)
    {
        if (parent[i] != i)
            parent[i] = find(parent[i]);
        return parent[i];
    }
    void unite(int i, int j)
    {
        int ri = find(i);
        int rj = find(j);
        if (ri != rj)
            parent[rj] = ri;
    }
};

// Build the Minimum Spanning Tree (MST) over the mutual reachability graph using Kruskal's algorithm.
static std::vector<r_edge> _buildMST(const std::vector<r_point> &points)
{
    int n = points.size();
    std::vector<r_edge> edges;
    // Construct all possible edges (O(n^2)); for large datasets a spatial index is advised.
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            double w = _mutualReachabilityDistance(points[i], points[j]);
            edges.push_back({i, j, w});
        }
    }
    std::sort(edges.begin(), edges.end(), [](const r_edge &e1, const r_edge &e2) {
        return e1.weight < e2.weight;
    });

    // Kruskal's algorithm to select edges for the MST.
    union_find uf(n);
    std::vector<r_edge> mst;
    for (const auto &edge : edges)
    {
        if (uf.find(edge.a) != uf.find(edge.b))
        {
            uf.unite(edge.a, edge.b);
            mst.push_back(edge);
        }
        if (mst.size() == static_cast<size_t>(n - 1))
            break;
    }
    return mst;
}

// Extract clusters from the MST by removing edges with weight above a given threshold.
// This is a simplified extraction compared to the full HDBSCAN hierarchy condensation.
static void _extraceClusters(const std::vector<r_edge> &mst, int nPoints, double threshold, std::vector<int> &labels)
{
    union_find uf(nPoints);
    for (const auto &edge : mst)
    {
        if (edge.weight <= threshold)
            uf.unite(edge.a, edge.b);
    }
    labels.resize(nPoints);
    for (int i = 0; i < nPoints; ++i)
        labels[i] = uf.find(i);
}

// Simplified HDBSCAN implementation for an r_image (assumed to be GRAY8).
// minPts: minimum number of points to consider for core distance.
// clusterSelectionThreshold: threshold on the mutual reachability distance to decide cluster connectivity.
std::vector<std::vector<r_point>> r_motion::gray8_hdbscan(const r_image &image, int minPts, double clusterSelectionThreshold)
{
    std::vector<std::vector<r_point>> clusters;
    // Collect nonzero pixels from the image.
    std::vector<r_point> points;
    for (int y = 0; y < image.height; ++y)
    {
        for (int x = 0; x < image.width; ++x)
        {
            uint8_t value = (*image.data)[y * image.width + x];
            if (value > 0)
                points.push_back({x, y, 0, false, 0.0});
        }
    }
    if (points.empty())
        return clusters;

    // Step 1: Compute core distances for all points.
    _computeCoreDistances(points, minPts);

    // Step 2: Build the mutual reachability MST.
    std::vector<r_edge> mst = _buildMST(points);

    // Step 3: Extract clusters by thresholding MST edges.
    std::vector<int> labels;
    _extraceClusters(mst, points.size(), clusterSelectionThreshold, labels);

    // Group points into clusters.
    std::unordered_map<int, std::vector<r_point>> clusterMap;
    for (size_t i = 0; i < points.size(); ++i)
        clusterMap[labels[i]].push_back(points[i]);
    
    for (auto &entry : clusterMap)
        clusters.push_back(entry.second);

    return clusters;
}