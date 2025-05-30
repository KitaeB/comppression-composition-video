#include <aom/aom_codec.h>
#include <zconf.h>
#include <zlib.h>

#include <cstdint>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "compression.h"

#pragma region common

cv::Mat frameSubstraction(const cv::Mat& frame, const cv::Mat& old_frame) {
<<<<<<< HEAD
  cv::Mat delta;
  cv::subtract(frame, old_frame, delta, cv::noArray(),
               CV_16SC3);  // Сначала вычисляем в CV_16SC3

  return delta;
=======
    cv::Mat delta;
    cv::subtract(frame, old_frame, delta, cv::noArray(), CV_16SC3);  // Сначала вычисляем в CV_16SC3

    return delta;
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
}

// Конвертируем в чистые данные Char
void convertToCleanDataChar(const cv::Mat& frame, std::vector<char>& data) {
<<<<<<< HEAD
  if (frame.isContinuous()) {
    data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
  } else {
    for (int i = 0; i < frame.rows; ++i) {
      data.insert(data.end(), frame.ptr<char>(i), frame.ptr<char>(i) + frame.cols * frame.elemSize());
=======
    if (frame.isContinuous()) {
        data.assign(frame.data, frame.data + frame.total() * frame.elemSize());
    } else {
        for (int i = 0; i < frame.rows; ++i) {
            data.insert(data.end(), frame.ptr<char>(i), frame.ptr<char>(i) + frame.cols * frame.elemSize());
        }
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
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
<<<<<<< HEAD
  lz4Stream = LZ4_createStream();
  dictBuffer.resize(DICT_SIZE);
=======
    lz4Stream = LZ4_createStream();
    dictBuffer.resize(DICT_SIZE);
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
}
LZ4Coder::~LZ4Coder() { LZ4_freeStream(lz4Stream); }

void LZ4Coder::convertToCleanDataChar() {
<<<<<<< HEAD
  if (inputFrame.isContinuous()) {
    uncompressedData.assign(inputFrame.data, inputFrame.data + inputFrame.total() * inputFrame.elemSize());
  } else {
    for (int i = 0; i < inputFrame.rows; ++i) {
      uncompressedData.insert(uncompressedData.end(), inputFrame.ptr<char>(i),
                              inputFrame.ptr<char>(i) + inputFrame.cols * inputFrame.elemSize());
=======
    if (inputFrame.isContinuous()) {
        uncompressedData.assign(inputFrame.data, inputFrame.data + inputFrame.total() * inputFrame.elemSize());
    } else {
        for (int i = 0; i < inputFrame.rows; ++i) {
            uncompressedData.insert(uncompressedData.end(), inputFrame.ptr<char>(i),
                                    inputFrame.ptr<char>(i) + inputFrame.cols * inputFrame.elemSize());
        }
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
    }
  }
  uncompressedSize = uncompressedData.size();
}

int LZ4Coder::lz4_compress_fast(cv::Mat& frame) {
<<<<<<< HEAD
  inputFrame = frame.clone();
  uncompressedSize = inputFrame.elemSize() * inputFrame.total();

  // Изменяем размер вектора сжатых данных
  compressedData.resize(LZ4_compressBound(uncompressedSize));
=======
    inputFrame = frame.clone();
    uncompressedSize = inputFrame.elemSize() * inputFrame.total();
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0

  if (!dictBuffer.empty()) LZ4_loadDict(lz4Stream, dictBuffer.data(), dictBuffer.size());

<<<<<<< HEAD
  compressedSize = LZ4_compress_fast_continue(lz4Stream, (const char*)inputFrame.data, compressedData.data(),
                                              uncompressedSize, compressedData.size(), acceleration);
  // Сохраняем словарь
  LZ4_saveDict(lz4Stream, dictBuffer.data(), dictBuffer.size());
=======
    if (!dictBuffer.empty()) LZ4_loadDict(lz4Stream, dictBuffer.data(), dictBuffer.size());
    compressedSize = LZ4_compress_fast_continue(lz4Stream, (const char*)inputFrame.data, compressedData.data(),
                                                uncompressedSize, compressedData.size(), acceleration);
    // Сохраняем словарь
    LZ4_saveDict(lz4Stream, dictBuffer.data(), dictBuffer.size());
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0

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
<<<<<<< HEAD
  int inputSize = input.size();
  compressed.resize(LZ4_compressBound(inputSize));

  int compressedSize = LZ4_compress_fast_continue(lz4Stream, input.data(), compressed.data(), inputSize,
                                                  compressed.size(), acceleration);
=======
    int inputSize = input.size();
    compressed.resize(LZ4_compressBound(inputSize));

    int compressedSize = LZ4_compress_fast_continue(lz4Stream, input.data(), compressed.data(), inputSize,
                                                    compressed.size(), acceleration);
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0

  if (compressedSize <= 0) return -1;
  compressed.resize(compressedSize);
  return compressedSize;
}

#pragma endregion

#pragma region zlib

ZLIBCoder::ZLIBCoder() {
    zStream.zalloc = Z_NULL;
    zStream.zfree = Z_NULL;
    zStream.opaque = Z_NULL;

<<<<<<< HEAD
    deflateInit(&zStream, Z_DEFAULT_COMPRESSION);
    zBuffer.resize(16384);
}

ZLIBCoder::~ZLIBCoder() { deflateEnd(&zStream); }

int ZLIBCoder::zlib_compress_stream(cv::Mat& frame) {
  inputFrame = frame.clone();
  compressedData.clear();

  uncompressedData = *(const std::vector<Bytef>*) inputFrame.data;
  originalSize = uncompressedData.size();

  zStream.avail_in = originalSize;
  zStream.next_in = uncompressedData.data();
  
  do {
    zStream.avail_out = zBuffer.size();
    zStream.next_out = zBuffer.data();

    int ret = deflate(&zStream, Z_NO_FLUSH);
    if (ret != Z_OK && ret != Z_STREAM_END) break;

    compressedSize = zBuffer.size() - zStream.avail_out;
    compressedData.insert(compressedData.end(), zBuffer.data(), zBuffer.data() + compressedSize);
  } while (zStream.avail_out == 0);

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
=======
    int result = compress(compressed_data.data(), &compressedSize, input.data(), input.size());
    if (result != Z_OK) {
        return -1;  // Ошибка сжатия
    }
    compressed_data.resize(compressedSize);
    return compressedSize;
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
}

// Функция для сжатия с быстрым режимом (Z_BEST_SPEED)
int zlib_compress_fast(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data) {
  uLongf compressedSize = compressBound(input.size());
  compressed_data.resize(compressedSize);

<<<<<<< HEAD
  int result = compress2(compressed_data.data(), &compressedSize, input.data(), input.size(), Z_BEST_SPEED);
  if (result != Z_OK) {
    return -1;  // Ошибка сжатия
  }
  compressed_data.resize(compressedSize);
  return compressedSize;
=======
    int result = compress2(compressed_data.data(), &compressedSize, input.data(), input.size(), Z_BEST_SPEED);
    if (result != Z_OK) {
        return -1;  // Ошибка сжатия
    }
    compressed_data.resize(compressedSize);
    return compressedSize;
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
}

// Функция для сжатия с максимальным уровнем компрессии (Z_BEST_COMPRESSION)
int zlib_compress_high(const std::vector<Bytef>& input, std::vector<Bytef>& compressed_data) {
  uLongf compressedSize = compressBound(input.size());
  compressed_data.resize(compressedSize);

<<<<<<< HEAD
  int result = compress2(compressed_data.data(), &compressedSize, input.data(), input.size(), Z_BEST_COMPRESSION);
  if (result != Z_OK) {
    return -1;  // Ошибка сжатия
  }
  compressed_data.resize(compressedSize);
  return compressedSize;
=======
    int result = compress2(compressed_data.data(), &compressedSize, input.data(), input.size(), Z_BEST_COMPRESSION);
    if (result != Z_OK) {
        return -1;  // Ошибка сжатия
    }
    compressed_data.resize(compressedSize);
    return compressedSize;
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
}

#pragma endregion

#pragma region zstd

<<<<<<< HEAD
aom_codec_ctx_t create_aom_encoder(const cv::Mat& frame, bool lossless = false, int target_fps = 30,
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
    // cfg.g_lossless = 1;   нет такого парамаметра
    cfg.rc_target_bitrate = 0;
    cfg.rc_min_quantizer = 0;
    cfg.rc_max_quantizer = 0;
  } else {
    // Режим с потерями, рассчитываем битрейт на основе bpp
    int bitrate_kbps = static_cast<int>(width * height * target_fps * bpp / 1000.0);
    cfg.rc_target_bitrate = bitrate_kbps;

    // Рекомендованные квантизаторы
    cfg.rc_min_quantizer = 10;  // Лучше качество — меньше значение
    cfg.rc_max_quantizer = 40;
  }
  if (aom_codec_enc_init(&encoder, aom_codec_av1_cx(), &cfg, 0)) throw std::runtime_error("Encoder init failed");

  if (lossless) {
    if (aom_codec_control(&encoder, AV1E_SET_LOSSLESS, 1)) throw std::runtime_error("AV1E_SET_LOSSLESS failed");

    if (aom_codec_control(&encoder, AOME_SET_CPUUSED, 8)) throw std::runtime_error("AOME_SET_CPUUSED failed");
  }
  encoder.name = "enc";
  return encoder;
}

int aom_compress_loseless(const cv::Mat& frame, std::vector<uint8_t>& compressed_data, aom_codec_ctx_t& encoder,
                          int frame_count) {
  const int width = frame.cols;
  const int height = frame.rows;

  if (frame.empty() || width <= 0 || height <= 0 || width % 2 != 0 || height % 2 != 0) {
    throw std::invalid_argument("Invalid frame dimensions");
  }

  // 1. Конвертируем BGR → YUV420
  cv::Mat yuv_img;
  cv::cvtColor(frame, yuv_img, cv::COLOR_BGR2YUV_I420);
  if (!yuv_img.isContinuous() || yuv_img.total() != width * height * 3 / 2) {
    throw std::runtime_error("Unexpected YUV layout");
  }

  // 2. Создаём изображение AOM (с внутренним буфером)
  aom_image_t raw;
  if (!aom_img_alloc(&raw, AOM_IMG_FMT_I420, width, height, 1)) {
    throw std::runtime_error("aom_img_alloc failed");
  }

  // 3. Копируем данные из YUV в raw.planes[0/1/2]
  const int y_size = width * height;
  const int uv_size = y_size / 4;

  std::memcpy(raw.planes[0], yuv_img.data, y_size);                      // Y
  std::memcpy(raw.planes[1], yuv_img.data + y_size, uv_size);            // U
  std::memcpy(raw.planes[2], yuv_img.data + y_size + uv_size, uv_size);  // V

  // 4. Кодируем
  compressed_data.clear();
  if (aom_codec_encode(&encoder, &raw, frame_count, 1, 0)) {
    std::string err = aom_codec_error(&encoder);
    std::string detail = aom_codec_error_detail(&encoder);
    aom_img_free(&raw);
    throw std::runtime_error("aom_codec_encode failed: " + err + " — " + detail);
  }

  // 5. Получаем результат
  aom_codec_iter_t iter = nullptr;
  const aom_codec_cx_pkt_t* pkt;
  while ((pkt = aom_codec_get_cx_data(&encoder, &iter))) {
    if (pkt->kind == AOM_CODEC_CX_FRAME_PKT) {
      const uint8_t* data = static_cast<const uint8_t*>(pkt->data.frame.buf);
      compressed_data.insert(compressed_data.end(), data, data + pkt->data.frame.sz);
    }
  }

  // 6. Освобождаем ресурсы
  aom_img_free(&raw);
  return static_cast<int>(compressed_data.size());
}

=======
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
>>>>>>> 1584364e37163758f38092b71ad308170fe9d5d0
#pragma endregion
