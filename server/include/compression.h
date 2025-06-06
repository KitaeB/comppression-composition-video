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
cv::Mat MatSub (const cv::Mat& new_frame, const cv::Mat& old_frame);


void convertToCleanDataBytef(const cv::Mat& frame, std::vector<Bytef>& data);
void point(int num);
/* ============================================================================================================ */
// Объявляем класс для сжатия библиотекой LZ4
class LZ4Coder {
public:
    // Конструктор и деструктор
    LZ4Coder();
    ~LZ4Coder();
    // Параметры входные
    cv::Mat inputFrame;

    // Параметры выходные
    std::vector<char> compressedData;
    int uncompressedSize, compressedSize, acceleration = 1;

    // Методы сжатия LZ4
    int lz4_compress_fast(cv::Mat& frame);
    int lz4_compress_fast_dict(cv::Mat& frame);

private:
    LZ4_stream_t* lz4Stream = nullptr;
    const int DICT_SIZE = 64 * 1024;
    std::vector<char> uncompressedData, dictBuffer;

    void convertToCleanDataChar();
};

class ZLIBCoder {
public:
    // Конструктор и деструктор
    ZLIBCoder();
    ~ZLIBCoder();

    // Входные параметры
    cv::Mat inputFrame;

    // Выходные параметры
    std::vector<Bytef> compressedData, uncompressedData;
    int originalSize;
    uLongf compressedSize;
    // Методы сжатия
    int zlib_compress_stream(cv::Mat& frame);

    // int zlib_compress_default(cv::Mat& frame);
    int zlib_compress_fast(cv::Mat& frame);
    // int zlib_compress_high(cv::Mat& frame);

private:
    // Поток данных
    z_stream zStream;

    // Выходной буффер
    std::vector<Bytef> zBuffer;
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
    int originalSize, compressedSize;
    cv::Mat inputFrame;


    // Метод сжатия в потоке
    bool zstd_compress_stream(cv::Mat frame);
    bool zstd_compress(cv::Mat frame);

private:
    ZSTD_CStream* cstream;
    ZSTD_inBuffer cBuffer;
};
