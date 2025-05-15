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

//Конвертируем в чистые данные Char
void convertToCleanDataChar(const cv::Mat& frame, std::vector<char>& data) {
    if (frame.isContinuous()) {
        data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
    } else {
        for (int i = 0; i < frame.rows; ++i) {
            data.insert(data.end(), frame.ptr<char>(i), frame.ptr<char>(i) + frame.cols * frame.elemSize());
        }
    }
}
//Конвертируем в чистые данные Bytef
void convertToCleanDataBytef(const cv::Mat& frame, std::vector<Bytef>& data) {
    if (frame.isContinuous()) {
        data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
    } else {
        for (int i = 0; i < frame.rows; ++i) {
            data.insert(data.end(), frame.ptr<Bytef>(i), frame.ptr<Bytef>(i) + frame.cols * frame.elemSize());
        }
    }
}

#pragma endregion

#pragma region lz4

// Сжатие LZ4 (default)
int lz4_compress_default(const std::vector<char>& input, std::vector<char>& compressed) {
    int inputSize = input.size();
    compressed.resize(LZ4_compressBound(inputSize));

    int compressedSize = LZ4_compress_default(
        input.data(),
        compressed.data(),
        inputSize,
        compressed.size()
    );

    if (compressedSize <= 0) return -1;
    compressed.resize(compressedSize);
    return compressedSize;
}

// Сжатие LZ4 (fast)
int lz4_compress_fast(const std::vector<char>& input, std::vector<char>& compressed, int acceleration) {
    int inputSize = input.size();
    compressed.resize(LZ4_compressBound(inputSize));

    int compressedSize = LZ4_compress_fast(
        input.data(),
        compressed.data(),
        inputSize,
        compressed.size(),
        acceleration
    );

    if (compressedSize <= 0) return -1;
    compressed.resize(compressedSize);
    return compressedSize;
}

#pragma endregion

#pragma region zlib

// Функция для сжатия с использованием стандартного уровня (Z_DEFAULT_COMPRESSION)
int zlib_compress_default(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data) {
    uLongf compressedSize = compressBound(input.size());
    compressed_data.resize(compressedSize);

    int result = compress(compressed_data.data(), &compressedSize, input.data(), input.size());
    if (result != Z_OK) {
        return -1; // Ошибка сжатия
    }
    compressed_data.resize(compressedSize);
    return compressedSize;
}

// Функция для сжатия с быстрым режимом (Z_BEST_SPEED)
int zlib_compress_fast(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data) {
    uLongf compressedSize = compressBound(input.size());
    compressed_data.resize(compressedSize);

    int result = compress2(compressed_data.data(), &compressedSize, input.data(), input.size(), Z_BEST_SPEED);
    if (result != Z_OK) {
        return -1; // Ошибка сжатия
    }
    compressed_data.resize(compressedSize);
    return compressedSize;
}

// Функция для сжатия с максимальным уровнем компрессии (Z_BEST_COMPRESSION)
int zlib_compress_high(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data) {
    uLongf compressedSize = compressBound(input.size());
    compressed_data.resize(compressedSize);

    int result = compress2(compressed_data.data(), &compressedSize, input.data(), input.size(), Z_BEST_COMPRESSION);
    if (result != Z_OK) {
        return -1; // Ошибка сжатия
    }
    compressed_data.resize(compressedSize);
    return compressedSize;
}

#pragma endregion