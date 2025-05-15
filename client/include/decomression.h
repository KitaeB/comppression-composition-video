#pragma once

#include <lz4.h>
#include <zlib.h>
//#include <aom.h>
#include <vector>
#include <opencv2/opencv.hpp>

//из вектора в Mat
cv::Mat convertFromCleanDataChar(const std::vector<char>& data, int rows, int cols, int type);

//из вектора в Mat
cv::Mat convertFromCleanDataBytef(const std::vector<Bytef>& data, int rows, int cols, int type);

//Сложение кадров
cv::Mat frameAddiiton(const cv::Mat& new_frame, const cv::Mat& old_frame);

// Распаковка lz4
bool lz4_decompress(const std::vector<char>& compressed, std::vector<char>& output, int originalSize);

// Распаковка zlib
int zlib_decompress(const std::vector<Bytef>& compressed, std::vector<Bytef>& output, int originalSize);