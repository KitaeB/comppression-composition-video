#include <decomression.h>
#include <zlib.h>
#include <opencv2/core/mat.hpp>

#pragma region common

cv::Mat frameAddiiton(const cv::Mat& diff, const cv::Mat& old_frame) {
    cv::Mat frame;
    cv::add(old_frame, diff, frame, cv::noArray(), CV_8UC3);
    return frame;
}

cv::Mat convertFromCleanDataChar(const std::vector<char>& data, int rows, int cols, int type) {
    // Создаём cv::Mat с указанными размерами и типом
    cv::Mat frame(rows, cols, type);

    // Проверяем, что размер данных соответствует ожидаемому
    size_t expected_size = rows * cols * frame.elemSize();
    if (data.size() != expected_size) {
        throw std::invalid_argument("Data size does not match expected Mat size");
    }

    // Копируем данные в cv::Mat
    if (frame.isContinuous()) {
        std::memcpy(frame.data, data.data(), data.size());
    } else {
        size_t offset = 0;
        for (int i = 0; i < rows; ++i) {
            char* row_ptr = frame.ptr<char>(i);
            size_t row_size = cols * frame.elemSize();
            std::memcpy(row_ptr, data.data() + offset, row_size);
            offset += row_size;
        }
    }

    return frame;
}

cv::Mat convertFromCleanDataBytef(const std::vector<Bytef>& data, int rows, int cols, int type) {
    // Создаём cv::Mat с указанными размерами и типом
    cv::Mat frame(rows, cols, type);

    // Проверяем, что размер данных соответствует ожидаемому
    size_t expected_size = rows * cols * frame.elemSize();
    if (data.size() != expected_size) {
        throw std::invalid_argument("Data size does not match expected Mat size");
    }

    // Копируем данные в cv::Mat
    if (frame.isContinuous()) {
        std::memcpy(frame.data, data.data(), data.size());
    } else {
        size_t offset = 0;
        for (int i = 0; i < rows; ++i) {
            Bytef* row_ptr = frame.ptr<Bytef>(i);
            size_t row_size = cols * frame.elemSize();
            std::memcpy(row_ptr, data.data() + offset, row_size);
            offset += row_size;
        }
    }

    return frame;
}

#pragma endregion

#pragma region lz4

LZ4Decoder::LZ4Decoder() { decoder = LZ4_createStreamDecode(); }

LZ4Decoder::~LZ4Decoder() { LZ4_freeStreamDecode(decoder); }

void LZ4Decoder::convertFromCleanDataChar() {
    // Проверяем, что размер данных соответствует ожидаемому
    size_t expected_size = outputFrame.elemSize() * outputFrame.total();
    if (decompressedData.size() != expected_size) {
        throw std::invalid_argument("Data size does not match expected Mat size");
    }

    // Копируем данные в cv::Mat
    if (outputFrame.isContinuous()) {
        std::memcpy(outputFrame.data, decompressedData.data(), decompressedData.size());
    } else {
        size_t offset = 0;
        for (int i = 0; i < outputFrame.rows; ++i) {
            char* row_ptr = outputFrame.ptr<char>(i);
            size_t row_size = outputFrame.cols * outputFrame.elemSize();
            std::memcpy(row_ptr, decompressedData.data() + offset, row_size);
            offset += row_size;
        }
    }
}

bool LZ4Decoder::lz4_decompress() {
    // Устанавливаем словарь (если есть предыдущий блок)
    if (!prevBlock.empty()) {
        LZ4_setStreamDecode(decoder, prevBlock.data(), prevBlock.size());
    }
    decompressedData.resize(originalSize);

    decompressedSize = LZ4_decompress_safe_continue(decoder, compressedData.data(), decompressedData.data(),
                                                    compressedData.size(), originalSize);
    if (decompressedSize <= 0) std::cerr << "Ошибка декомпрессии!" << std::endl;
    // Обновляем словарь: сохраняем последние 64 KB данных
    prevBlock.assign(decompressedData.data() + decompressedSize - std::min(decompressedSize, 512 * 1024),
                     decompressedData.data() + decompressedSize);

    convertFromCleanDataChar();
    return decompressedSize > 0;
}

// Распаковка lz4
bool lz4_decompress(const std::vector<char>& compressed, std::vector<char>& output, int originalSize) {
    output.resize(originalSize);

    int result = LZ4_decompress_safe(compressed.data(), output.data(), compressed.size(), originalSize);

    return result >= 0;
}

#pragma endregion

#pragma region zlib

ZLIBDecoder::ZLIBDecoder() {
    zStream.zalloc = Z_NULL;
    zStream.zfree = Z_NULL;
    zStream.opaque = Z_NULL;

    inflateInit(&zStream);
    zBuffer.resize(16384);
}

ZLIBDecoder::~ZLIBDecoder() { inflateEnd(&zStream); }

void ZLIBDecoder::convertFromCleanDataBytef() {

    // Проверяем, что размер данных соответствует ожидаемому
    size_t expected_size = outputFrame.total() * outputFrame.elemSize();
    if (decompressedData.size() != expected_size) {
        throw std::invalid_argument("Data size does not match expected Mat size");
    }

    // Копируем данные в cv::Mat
    if (outputFrame.isContinuous()) {
        std::memcpy(outputFrame.data, decompressedData.data(), decompressedData.size());
    } else {
        size_t offset = 0;
        for (int i = 0; i < outputFrame.rows; ++i) {
            Bytef* row_ptr = outputFrame.ptr<Bytef>(i);
            size_t row_size = outputFrame.cols * outputFrame.elemSize();
            std::memcpy(row_ptr, decompressedData.data() + offset, row_size);
            offset += row_size;
        }
    }
}


bool ZLIBDecoder::zlib_decompress_stream() {
    zStream.avail_in = compressedSize;
    zStream.next_in = compressedData.data();

    decompressedData.clear();

    do {
        zStream.avail_out = zBuffer.size();
        zStream.next_out = zBuffer.data();

        int ret = inflate(&zStream, Z_NO_FLUSH);  // Поддержка зависимости от предыдущих кадров
        if (ret != Z_OK && ret != Z_STREAM_END) {
            return false;
        }

        decompressedSize = zBuffer.size() - zStream.avail_out;
        decompressedData.insert(decompressedData.end(), zBuffer.data(), zBuffer.data() + decompressedSize);
    } while (zStream.avail_out == 0);
    
    convertFromCleanDataBytef();

    return !decompressedData.empty();
}

int zlib_decompress(const std::vector<Bytef>& compressed_data, std::vector<Bytef>& output_data, int original_size) {
    output_data.resize(original_size);
    uLongf decompressed_size = original_size;

    int result = uncompress(output_data.data(), &decompressed_size, compressed_data.data(), compressed_data.size());
    if (result != Z_OK) {
        return -1;  // Ошибка распаковки
    }

    return 0;  // Успешная распаковка
}

#pragma endregion

#pragma region aom

bool aom_decompress(const std::vector<uint8_t>& encoded_data, cv::Mat& frame) {
    if (encoded_data.empty()) {
        std::cerr << "Encoded data is empty\n";
        return false;
    }

    aom_codec_ctx_t decoder = {};
    if (aom_codec_dec_init(&decoder, aom_codec_av1_dx(), nullptr, 0)) {
        std::cerr << "aom_codec_dec_init failed\n";
        return false;
    }

    if (aom_codec_decode(&decoder, encoded_data.data(), encoded_data.size(), nullptr)) {
        std::cerr << "aom_codec_decode failed\n";
        aom_codec_destroy(&decoder);
        return false;
    }

    aom_codec_iter_t iter = nullptr;
    aom_image_t* img = aom_codec_get_frame(&decoder, &iter);
    if (!img) {
        std::cerr << "No frame was decoded\n";
        aom_codec_destroy(&decoder);
        return false;
    }

    // Проверим формат изображения
    if (img->fmt != AOM_IMG_FMT_I420) {
        std::cerr << "Unsupported image format: " << img->fmt << "\n";
        aom_codec_destroy(&decoder);
        return false;
    }

    int width = img->d_w;
    int height = img->d_h;

    // Создаем Y, U, V
    cv::Mat y(height, width, CV_8UC1, img->planes[0], img->stride[0]);
    cv::Mat u(height / 2, width / 2, CV_8UC1, img->planes[1], img->stride[1]);
    cv::Mat v(height / 2, width / 2, CV_8UC1, img->planes[2], img->stride[2]);

    // Масштабируем UV до размера Y
    cv::Mat u_resized, v_resized;
    cv::resize(u, u_resized, y.size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(v, v_resized, y.size(), 0, 0, cv::INTER_LINEAR);

    // Объединяем каналы и преобразуем в BGR
    std::vector<cv::Mat> channels = {y, u_resized, v_resized};
    cv::Mat yuv, bgr;
    cv::merge(channels, yuv);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR);

    frame = bgr.clone();

    aom_codec_destroy(&decoder);
    return true;
}

#pragma endregion
