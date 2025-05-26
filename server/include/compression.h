#pragma once

#include <lz4.h>

#include <zlib.h>

#include <aom/aom_codec.h>
#include <aom/aom_encoder.h>
#include <aom/aomcx.h>

#include <vector>
#include <opencv2/opencv.hpp>

//Общие методы для работы с изорбражениями
cv::Mat frameSubstraction(const cv::Mat& frame, const cv::Mat& old_frame);
void convertToCleanDataChar(const cv::Mat& frame, std::vector<char>& data);
void convertToCleanDataBytef(const cv::Mat& frame, std::vector<Bytef>& data);

//Методы сжатия LZ4
int lz4_compress_default(const std::vector<char>& input, std::vector<char>& compressed);
int lz4_compress_fast(const std::vector<char>& input, std::vector<char>& compressed, int acceleration = 1);

//Методы сждатия zlib
int zlib_compress_default(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);
int zlib_compress_fast(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);
int zlib_compress_high(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);

//Методы сжатия aom
aom_codec_ctx_t create_aom_encoder(const cv::Mat& frame, bool lossless, int target_fps, double bpp);  // bits per pixel (только для сжатия с потерями)
int aom_compress_loseless(const cv::Mat& frame, std::vector<uint8_t>& compressed_data, aom_codec_ctx_t& encoder, int frame_count);
