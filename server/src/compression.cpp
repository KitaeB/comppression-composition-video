#include <aom/aom_codec.h>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <vector>

#include "compression.h"

#pragma region common

cv::Mat frameSubstraction(const cv::Mat& frame, const cv::Mat& old_frame){
    cv::Mat delta;
    cv::subtract(frame, old_frame, delta, cv::noArray(), CV_16SC3);  // Сначала вычисляем в CV_16SC3
    
    return delta;

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

#pragma region aom

aom_codec_ctx_t create_aom_encoder(
    const cv::Mat& frame,
    bool lossless = false,
    int target_fps = 30,
    double bpp = 0.1  // bits per pixel (только для сжатия с потерями)
) {
    if (frame.empty()) {
        throw std::invalid_argument("select_aom_codec_config: пустое изображение");
    }

    const int width = frame.cols;
    const int height = frame.rows;
    aom_codec_ctx_t encoder;

    // Получаем конфигурацию по умолчанию
    aom_codec_enc_cfg_t cfg;
    if (aom_codec_enc_config_default(aom_codec_av1_cx(), &cfg, 0)) {
        throw std::runtime_error("aom_codec_enc_config_default failed");
    }

    // Базовые параметры
    cfg.g_w = width;
    cfg.g_h = height;
    cfg.g_timebase.num = 1;
    cfg.g_timebase.den = target_fps;
    cfg.g_pass = AOM_RC_ONE_PASS;
    cfg.g_lag_in_frames = 0;

    if (lossless) {
        // Режим без потерь
        //cfg.g_lossless = 1;   нет такого парамаметра
        cfg.rc_target_bitrate = 0;
        cfg.rc_min_quantizer = 0;
        cfg.rc_max_quantizer = 0;
    } else {
        // Режим с потерями, рассчитываем битрейт на основе bpp
        int bitrate_kbps = static_cast<int>(width * height * target_fps * bpp / 1000.0);
        cfg.rc_target_bitrate = bitrate_kbps;

        // Рекомендованные квантизаторы
        cfg.rc_min_quantizer = 10;   // Лучше качество — меньше значение
        cfg.rc_max_quantizer = 40;
    }
    if (aom_codec_enc_init(&encoder, aom_codec_av1_cx(), &cfg, 0))
        throw std::runtime_error("Encoder init failed");

    if (lossless) {
        if (aom_codec_control(&encoder, AV1E_SET_LOSSLESS, 1))
            throw std::runtime_error("AV1E_SET_LOSSLESS failed");

        if (aom_codec_control(&encoder, AOME_SET_CPUUSED, 8))
            throw std::runtime_error("AOME_SET_CPUUSED failed");    }
    encoder.name = "enc";
    return encoder;
}

int aom_compress_loseless(const cv::Mat& frame, std::vector<uint8_t>& compressed_data, aom_codec_ctx_t& encoder) {
    const int width = frame.cols;
    const int height = frame.rows;
    
    // 1. Преобразуем в YUV420
    cv::Mat yuv_img;
    cv::cvtColor(frame, yuv_img, cv::COLOR_BGR2YUV_I420);
    std::vector<uint8_t> yuv_data(yuv_img.data, yuv_img.data + yuv_img.total());

    // 2. Оборачиваем YUV буфер в aom_image_t
    aom_image_t raw;
    if (!aom_img_wrap(&raw, AOM_IMG_FMT_I420, width, height, 1, yuv_data.data())) {
        throw std::runtime_error("aom_img_wrap failed");
    }

    // 3. Кодируем один кадр
    if (aom_codec_encode(&encoder, &raw, 0, 1, 0)) {
        throw std::runtime_error("aom_codec_encode failed");
    }

    // 4. Получаем закодированные данные
    aom_codec_iter_t iter = nullptr;
    const aom_codec_cx_pkt_t* pkt;

    while ((pkt = aom_codec_get_cx_data(&encoder, &iter))) {
        if (pkt->kind == AOM_CODEC_CX_FRAME_PKT) {
            const uint8_t* data = static_cast<const uint8_t*>(pkt->data.frame.buf);
            compressed_data.insert(compressed_data.end(), data, data + pkt->data.frame.sz);
        }
    }
    return compressed_data.size();
}

#pragma endregion