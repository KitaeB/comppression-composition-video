#pragma once

#include <opencv2/core/mat.hpp>

#include <lz4.h>

#include <zlib.h>

#include <zstd.h>

#include <vector>
#include <opencv2/opencv.hpp>

/* ============================================================================================================ */
// Общие методы для работы с изорбражениями
cv::Mat frameSubstraction(const cv::Mat& frame, const cv::Mat& old_frame);
void convertToCleanDataBytef(const cv::Mat& frame, std::vector<Bytef>& data);

/* ============================================================================================================ */
// Объявляем класс для сжатия библиотекой LZ4
class LZ4Coder {
public:
    LZ4Coder();
    ~LZ4Coder();
    // Параметры входные
    cv::Mat inputFrame;

    // Параметры выходные
    std::vector<char> compressedData;
    int uncompressedSize, compressedSize, acceleration = 1;

    // Методы сжатия LZ4
    int lz4_compress_fast(cv::Mat& frame);

private:
    LZ4_stream_t* lz4Stream = nullptr;
    const int DICT_SIZE = 64 * 1024;
    std::vector<char> uncompressedData, dictBuffer;

    void convertToCleanDataChar();
};

/* ============================================================================================================ */
// Объявляем класс для сжатия библиотекой ZLIB
class ZLIBCoder {
public:
    ZLIBCoder();
    ~ZLIBCoder();
    // Входные параметры
    cv::Mat inputFrame;

    // Выходные параметры

private:
};

// Методы сждатия zlib
int zlib_compress_default(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);
int zlib_compress_fast(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);
int zlib_compress_high(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data);

/* ============================================================================================================ */
// Объявляем класс для сжатия библиотекой ZSTD
class ZSTDCoder {
public:
    // Конструктор и деструктор
    ZSTDCoder();
    ~ZSTDCoder();

    // Выходные данные
    std::vector<char> uncomressedData, compressedData;
    int uncompressedSize, compressedSize;

    // Метод сжатия в потоке
    bool compress_stream(cv::Mat frame);

private:
    ZSTD_CStream* cstream;
    ZSTD_inBuffer cBuffer;
    cv::Mat inputFrame;
};
