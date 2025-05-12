#pragma once

#include <lz4.h>
#include <zlib.h>
#include <vector>
#include <opencv2/opencv.hpp>

//Общие методы для работы с изорбражениями
cv::Mat frameSubstraction(const cv::Mat& new_frame, const cv::Mat& old_frame);
std::vector<char>& convertToCleanData(const cv::Mat& frame);

//Методы сжатия LZ4
int lz4_compress_default(const std::vector<char>& input, std::vector<char>& compressed);
int lz4_compress_fast(const std::vector<char>& input, std::vector<char>& compressed, int acceleration = 1);

//Методы сждатия zlib
int zlib_compress_default(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);
int zlib_compress_fast(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);
int zlib_compress_high(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);

//Методы сжатия AV1