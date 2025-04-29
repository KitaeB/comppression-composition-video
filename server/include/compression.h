#pragma once
#include <lz4.h>
#include <zlib.h>
#include <opencv2/opencv.hpp>

cv::Mat frameSubstraction(const cv::Mat& new_frame, const cv::Mat& old_frame);

