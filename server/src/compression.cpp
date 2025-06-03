#include <aom/aom_codec.h>
#include <zconf.h>
#include <zlib.h>

#include <cstring>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "compression.h"

#pragma region common

void point(int num) { std::cout << "point " << num << std::endl; }

cv::Mat frameSubstraction(const cv::Mat& new_frame, const cv::Mat& old_frame) {
    cv::Mat delta;
    cv::subtract(new_frame, old_frame, delta, cv::noArray(), CV_16SC3);  // Сначала вычисляем в CV_16SC3

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

    compressedSize = LZ4_compress_fast_continue(lz4Stream, (const char*)inputFrame.data, compressedData.data(), uncompressedSize,
                                                compressedData.size(), acceleration);
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
int lz4_compress_fast(LZ4_stream_t* lz4Stream, const std::vector<char>& input, std::vector<char>& compressed, int acceleration) {
    int inputSize = input.size();
    compressed.resize(LZ4_compressBound(inputSize));

    int compressedSize =
        LZ4_compress_fast_continue(lz4Stream, input.data(), compressed.data(), inputSize, compressed.size(), acceleration);

    if (compressedSize <= 0) return -1;
    compressed.resize(compressedSize);
    return compressedSize;
}

#pragma endregion

#pragma region zlib

// Использование потока не отработано и не работает
ZLIBCoder::ZLIBCoder() {
    zStream.zalloc = Z_NULL;
    zStream.zfree = Z_NULL;
    zStream.opaque = Z_NULL;

    if (deflateInit(&zStream, Z_BEST_SPEED) != Z_OK) std::cerr << "Error deflate init" << std::endl;

    zBuffer.resize(256 * 1024);
}

ZLIBCoder::~ZLIBCoder() { deflateEnd(&zStream); }

int ZLIBCoder::zlib_compress_stream(cv::Mat& frame) {
    inputFrame = frame.clone();
    compressedData.clear();

    const uchar* data = reinterpret_cast<const uchar*>(inputFrame.data);
    size_t size = inputFrame.total() * inputFrame.elemSize();

    uncompressedData.assign(data, data + size);
    originalSize = uncompressedData.size();

    zStream.avail_in = originalSize;
    zStream.next_in = uncompressedData.data();

    // Сброс потока для этой операции сжатия
    if (deflateReset(&zStream) != Z_OK) {
        std::cerr << "Rror deflate reset" << std::endl;
        return -1;
    }

    zStream.avail_in = originalSize;
    zStream.next_in = uncompressedData.data();

    int ret;
    do {
        zStream.avail_out = zBuffer.size();
        zStream.next_out = zBuffer.data();

        ret = deflate(&zStream, Z_FINISH);

        if (ret != Z_OK && ret != Z_STREAM_END) {
            std::cerr << "Error deflate: " << ret << std::endl;
            return -1;
        }

        size_t bytesCompressed = zBuffer.size() - zStream.avail_out;
        compressedData.insert(compressedData.end(), zBuffer.data(), zBuffer.data() + bytesCompressed);

    } while (ret != Z_STREAM_END);

    compressedSize = compressedData.size();
    return compressedSize;
}

int ZLIBCoder::zlib_compress_fast(cv::Mat& frame) {
    originalSize = frame.total() * frame.elemSize();

    uLongf compressedSize = compressBound(originalSize);
    compressedData.resize(compressedSize);

    int result = compress2(compressedData.data(), &compressedSize, frame.data, originalSize, Z_BEST_SPEED);

    if (result != Z_OK) {
        std::cerr << "compress error " << result << std::endl;
        return 0;  // Ошибка сжатия
    }
    return compressedSize;
}

// Функция для сжатия с использованием стандартного уровня
// (Z_DEFAULT_COMPRESSION)
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
    // Создаём контекст сжатия
    cstream = ZSTD_createCStream();
    // Выставляем уровень сжатия
    ZSTD_initCStream(cstream, 3);
}

ZSTDCoder::~ZSTDCoder() { ZSTD_freeCStream(cstream); }

bool ZSTDCoder::zstd_compress_stream(cv::Mat frame) {
    inputFrame = frame.clone();

    const char* data = reinterpret_cast<const char*>(inputFrame.data);
    size_t size = inputFrame.total() * inputFrame.elemSize();

    uncomressedData = std::vector<char>(data, data + size);
    originalSize = uncomressedData.size();

    size_t const buff_in_size = ZSTD_CStreamInSize();
    std::vector<char> buff_in(buff_in_size);
    size_t const buff_out_size = ZSTD_CStreamOutSize();
    std::vector<char> buff_out(buff_out_size);

    ZSTD_initCStream(cstream, 3); // Реинициализация контекста

    ZSTD_inBuffer in_buf = {uncomressedData.data(), uncomressedData.size(), 0};

    compressedData.clear(); // Очищаем перед новым сжатием

    while (in_buf.pos < in_buf.size) {
        size_t read_size = std::min(buff_in_size, in_buf.size - in_buf.pos);
        memcpy(buff_in.data(), (char*)in_buf.src + in_buf.pos, read_size);
        in_buf.pos += read_size;

        ZSTD_inBuffer tmp_in_buf = {buff_in.data(), read_size, 0};
        while (tmp_in_buf.pos < tmp_in_buf.size) {
            ZSTD_outBuffer out_buf = {buff_out.data(), buff_out_size, 0};
            size_t ret = ZSTD_compressStream2(cstream, &out_buf, &tmp_in_buf, ZSTD_e_flush);
            if (ZSTD_isError(ret)) {
                throw std::runtime_error(ZSTD_getErrorName(ret));
            }
            compressedData.insert(compressedData.end(), buff_out.data(), buff_out.data() + out_buf.pos);
        }
    }

    ZSTD_outBuffer out_buf = {buff_out.data(), buff_out_size, 0};
    size_t ret = ZSTD_endStream(cstream, &out_buf);
    if (ZSTD_isError(ret)) {
        throw std::runtime_error(ZSTD_getErrorName(ret));
    }
    compressedData.insert(compressedData.end(), buff_out.data(), buff_out.data() + out_buf.pos);
    compressedSize = compressedData.size();

    return compressedSize > 0;
}

bool ZSTDCoder::zstd_compress(cv::Mat frame) {
    inputFrame = frame.clone();

    const char* data = reinterpret_cast<const char*>(inputFrame.data);
    size_t size = inputFrame.total() * inputFrame.elemSize();

    uncomressedData = std::vector<char>(data, data + size);
    originalSize = size;

    // Создаём контекст
    ZSTD_CCtx* cctx = ZSTD_createCCtx();
    if (!cctx) throw std::runtime_error("Не удалось создать ZSTD_CCtx");

    // Настройка параметров
    ZSTD_CCtx_setParameter(cctx, ZSTD_c_compressionLevel, 1);  // можно 1 для скорости
    ZSTD_CCtx_setParameter(cctx, ZSTD_c_nbWorkers, 4); // для многопоточности

    // Выделяем буфер
    size_t max_compressed_size = ZSTD_compressBound(originalSize);
    compressedData.resize(max_compressed_size);

    // Сжимаем
    size_t compressedSize = ZSTD_compress2(
        cctx,
        compressedData.data(), max_compressed_size,
        uncomressedData.data(), originalSize
    );

    if (ZSTD_isError(compressedSize)) {
        ZSTD_freeCCtx(cctx);
        throw std::runtime_error(ZSTD_getErrorName(compressedSize));
    }

    compressedData.resize(compressedSize);
    this->compressedSize = compressedSize;

    ZSTD_freeCCtx(cctx);

    return compressedSize > 0;
}


#pragma endregion
