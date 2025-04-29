#include <lz4.h>
#include <zlib.h>
#include <opencv2/opencv.hpp>

#include "compression.h"

#pragma region common
cv::Mat frameSubstraction(const cv::Mat& new_frame, const cv::Mat& old_frame){
        // Преобразуем кадры в 16-битный формат со знаком (CV_16S)
    cv::Mat new_frame_16, old_frame_16;
    new_frame.convertTo(new_frame_16, CV_16S);
    old_frame.convertTo(old_frame_16, CV_16S);

    // Вычисляем разницу между кадрами
    cv::Mat diff = new_frame_16 - old_frame_16;

    // Преобразуем результат обратно в 8-битный формат
    cv::Mat result;
    diff.convertTo(result, CV_8U);

    return result;
}
#pragma endregion

#pragma region method_1

#pragma endregion