#pragma once

#include <cstdint>
#include <lz4.h>

#include <opencv2/core/mat.hpp>
#include <zlib.h>

#include <aom/aom_decoder.h>
#include <aom/aomdx.h>

#include <vector>
#include <opencv2/opencv.hpp>

//из вектора в Mat
cv::Mat convertFromCleanDataChar(const std::vector<char>& data, int rows, int cols, int type);

//из вектора в Mat
cv::Mat convertFromCleanDataBytef(const std::vector<Bytef>& data, int rows, int cols, int type);

//Сложение кадров
cv::Mat frameAddiiton(const cv::Mat& new_frame, const cv::Mat& old_frame);

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

        private:
            z_stream zStream;
            std::vector<Bytef> zBuffer;

            void convertFromCleanDataBytef();
};
// Распаковка zlib
int zlib_decompress(const std::vector<Bytef>& compressed, std::vector<Bytef>& output, int originalSize);

// Распаковка aom
bool aom_decompress(const std::vector<uint8_t>& encoded_data, cv::Mat& frame);