#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <kas_utils/multi_array.hpp>

namespace rtabmap {

using kas_utils::MultiArray;

std::string compressString(const std::string& uncompressed);
std::string decompressString(const std::string& compressed);

template <typename T, std::size_t Dims>
std::string compressMultiArray(const MultiArray<T, Dims>& array);
template <typename T, std::size_t Dims>
MultiArray<T, Dims> decompressMultiArray(const std::string& compressed);

// backward compatibility
std::string compressMat(const cv::Mat& mat);
cv::Mat decompressMat(const std::string& compressed);

}
