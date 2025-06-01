#pragma once

#include <lz4.h>

#include <opencv2/core/mat.hpp>
#include <zlib.h>

#include <zstd.h>

#include <aom/aom_decoder.h>
#include <aom/aomdx.h>

#include <vector>
#include <opencv2/opencv.hpp>

// из вектора в Mat
cv::Mat convertFromCleanDataChar(const std::vector<char>& data, int rows, int cols, int type);

// из вектора в Mat
cv::Mat convertFromCleanDataBytef(const std::vector<Bytef>& data, int rows, int cols, int type);

// Сложение кадров
cv::Mat frameAddiiton(const cv::Mat& new_frame, const cv::Mat& old_frame);

void point(int num);

// Распаковка lz4
class LZ4Decoder {
public:
    LZ4Decoder();
    ~LZ4Decoder();

    // Параметры входные
    std::vector<char> compressedData, decompressedData;
    int decompressedSize, compressedSize, originalSize;

    // Параметры выходные
    cv::Mat outputFrame;

    // Метод декомпрессии
    bool lz4_decompress();

private:
    LZ4_streamDecode_t* decoder = LZ4_createStreamDecode();
    std::vector<char> prevBlock;

    void convertFromCleanDataChar();
};

bool lz4_decompress(const std::vector<char>& compressed, std::vector<char>& output, int originalSize);

// Распаковка ZLIB
class ZLIBDecoder {
public:
    ZLIBDecoder();
    ~ZLIBDecoder();

    // Параметры входные
    std::vector<Bytef> compressedData, decompressedData;
    int decompressedSize, compressedSize, originalSize;

    // Параметры выходные
    cv::Mat outputFrame;

    // Метод декомпрессии
    bool zlib_decompress_stream();

    bool zlib_decompress();

private:
    z_stream zStream;
    std::vector<Bytef> zBuffer;

    void convertFromCleanDataBytef();
};

// Распаковка zlib
int zlib_decompress(const std::vector<Bytef>& compressed, std::vector<Bytef>& output, int originalSize);

// Распаковка ZSTD
class ZSTDDecoder {
public:
    ZSTDDecoder();
    ~ZSTDDecoder();

    // Параметры входные
    std::vector<Bytef> compressedData, decompressedData;
    int decompressedSize, compressedSize, originalSize;

    // Параметры выходные
    cv::Mat outputFrame;

    // Метод декомпрессии
    bool zstd_decompress_stream();

    
private:
    ZSTD_DStream* dstream;
    ZSTD_DCtx* dctx = ZSTD_createDCtx();


};

// Распаковка zlib
int zlib_decompress(const std::vector<Bytef>& compressed, std::vector<Bytef>& output, int originalSize);
