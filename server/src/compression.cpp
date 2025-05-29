#include <aom/aom_codec.h>
#include <cstdint>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "compression.h"

#pragma region common

cv::Mat frameSubstraction(const cv::Mat& frame, const cv::Mat& old_frame) {
    cv::Mat delta;
    cv::subtract(frame, old_frame, delta, cv::noArray(), CV_16SC3);  // Сначала вычисляем в CV_16SC3

    return delta;
}

// Конвертируем в чистые данные Char
void convertToCleanDataChar(const cv::Mat& frame, std::vector<char>& data) {
    if (frame.isContinuous()) {
        data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
    } else {
        for (int i = 0; i < frame.rows; ++i) {
            data.insert(data.end(), frame.ptr<char>(i), frame.ptr<char>(i) + frame.cols * frame.elemSize());
        }
    }
}
// Конвертируем в чистые данные Bytef
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

LZ4Coder::LZ4Coder() {
    lz4Stream = LZ4_createStream();
    dictBuffer.resize(DICT_SIZE);
}
LZ4Coder::~LZ4Coder() { LZ4_freeStream(lz4Stream); }

void LZ4Coder::convertToCleanDataChar() {
    if (inputFrame.isContinuous()) {
        uncompressedData.assign(inputFrame.data, inputFrame.data + inputFrame.total() * inputFrame.elemSize());
    } else {
        for (int i = 0; i < inputFrame.rows; ++i) {
            uncompressedData.insert(uncompressedData.end(), inputFrame.ptr<char>(i),
                                    inputFrame.ptr<char>(i) + inputFrame.cols * inputFrame.elemSize());
        }
    }
    uncompressedSize = uncompressedData.size();
}

int LZ4Coder::lz4_compress_fast(cv::Mat& frame) {
    inputFrame = frame.clone();
    uncompressedSize = inputFrame.elemSize() * inputFrame.total();

    // Изменяем размер вектора сжатых данных
    compressedData.resize(LZ4_compressBound(uncompressedSize));

    if (!dictBuffer.empty()) LZ4_loadDict(lz4Stream, dictBuffer.data(), dictBuffer.size());
    compressedSize = LZ4_compress_fast_continue(lz4Stream, (const char*)inputFrame.data, compressedData.data(),
                                                uncompressedSize, compressedData.size(), acceleration);
    // Сохраняем словарь
    LZ4_saveDict(lz4Stream, dictBuffer.data(), dictBuffer.size());

    // Проверяем успешность сжатия
    if (compressedSize <= 0) {
        std::cerr << "LZ4 compression failed!" << std::endl;
        compressedData.clear();
        return 0;
    }
    // Изменяем размер вектора
    compressedData.resize(this->compressedSize);
    // Возвращаем размер сжатых данных
    return compressedSize;
}

// Сжатие LZ4 (fast)
int lz4_compress_fast(LZ4_stream_t* lz4Stream, const std::vector<char>& input, std::vector<char>& compressed,
                      int acceleration) {
    int inputSize = input.size();
    compressed.resize(LZ4_compressBound(inputSize));

    int compressedSize = LZ4_compress_fast_continue(lz4Stream, input.data(), compressed.data(), inputSize,
                                                    compressed.size(), acceleration);

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
        return -1;  // Ошибка сжатия
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
        return -1;  // Ошибка сжатия
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
        return -1;  // Ошибка сжатия
    }
    compressed_data.resize(compressedSize);
    return compressedSize;
}

#pragma endregion

#pragma region zstd

ZSTDCoder::ZSTDCoder() {
    //Создаём контекст сжатия
    cstream = ZSTD_createCStream();
    // Выставляем уровень сжатия
    ZSTD_initCStream(cstream, 3);
}

ZSTDCoder::~ZSTDCoder() {
    ZSTD_freeCStream(cstream);
}

bool ZSTDCoder::compress_stream(cv::Mat frame) {
    inputFrame = frame.clone();

    const char* data = reinterpret_cast<const char*>(inputFrame.data);
    size_t size = inputFrame.total() * inputFrame.elemSize();

    uncomressedData = std::vector<char>(data, data + size);

// Буферы
    size_t const buff_in_size = ZSTD_CStreamInSize();
    std::vector<char> buff_in(buff_in_size);
    size_t const buff_out_size = ZSTD_CStreamOutSize();
    std::vector<char> buff_out(buff_out_size);

    // Входной буфер
    ZSTD_inBuffer in_buf = { uncomressedData.data(), uncomressedData.size(), 0 };
    size_t to_read = buff_in_size;

    while (in_buf.pos < in_buf.size) {
        // Копируем часть данных во временный буфер
        size_t read_size = std::min(to_read, in_buf.size - in_buf.pos);
        memcpy(buff_in.data(), (char*)in_buf.src + in_buf.pos, read_size);
        in_buf.pos += read_size;

        ZSTD_inBuffer tmp_in_buf = { buff_in.data(), read_size, 0 };
        while (tmp_in_buf.pos < tmp_in_buf.size) {
            ZSTD_outBuffer out_buf = { buff_out.data(), buff_out_size, 0 };
            size_t ret = ZSTD_compressStream(cstream, &out_buf, &tmp_in_buf);
            if (ZSTD_isError(ret)) {
                ZSTD_freeCStream(cstream);
                throw std::runtime_error(ZSTD_getErrorName(ret));
            }
            // Добавляем сжатые данные в результат
            compressedData.insert(compressedData.end(), buff_out.data(), buff_out.data() + out_buf.pos);
        }
    }

    // Завершаем сжатие
    ZSTD_outBuffer out_buf = { buff_out.data(), buff_out_size, 0 };
    size_t ret = ZSTD_endStream(cstream, &out_buf);
    if (ZSTD_isError(ret)) {
        ZSTD_freeCStream(cstream);
        throw std::runtime_error(ZSTD_getErrorName(ret));
    }
    compressedData.insert(compressedData.end(), buff_out.data(), buff_out.data() + out_buf.pos);
    compressedSize = compressedData.size();
    // Очистка
    ZSTD_freeCStream(cstream);
    return compressedSize > 0;
}
#pragma endregion
