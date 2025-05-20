#include <decomression.h> 
#include <opencv2/core/mat.hpp>

#pragma region common

cv::Mat frameAddiiton(const cv::Mat& diff, const cv::Mat& old_frame){
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

// Распаковка lz4
bool lz4_decompress(const std::vector<char>& compressed, std::vector<char>& output, int originalSize) {
    output.resize(originalSize);

    int result = LZ4_decompress_safe(
        compressed.data(),
        output.data(),
        compressed.size(),
        originalSize
    );

    return result >= 0;
}

#pragma endregion

#pragma region zlib

int zlib_decompress(const std::vector<Bytef>& compressed_data, std::vector<Bytef>& output_data, int original_size) {
    output_data.resize(original_size);
    uLongf decompressed_size = original_size;

    int result = uncompress(output_data.data(), &decompressed_size, compressed_data.data(), compressed_data.size());
    if (result != Z_OK) {
        return -1; // Ошибка распаковки
    }

    return 0; // Успешная распаковка
}

#pragma endregion

#pragma region aom

bool aom_decompress(const std::vector<uint8_t>& encoded_data, cv::Mat& frame) {
    aom_codec_ctx_t decoder;
    if (aom_codec_dec_init(&decoder, aom_codec_av1_dx(), nullptr, 0)) {
        throw std::runtime_error("aom_codec_dec_init failed");
        return false;
    }

    if (aom_codec_decode(&decoder, encoded_data.data(), encoded_data.size(), nullptr)) {
        throw std::runtime_error("aom_codec_decode failed");
        return false;
    }

    aom_codec_iter_t iter = nullptr;
    aom_image_t* img = aom_codec_get_frame(&decoder, &iter);

    if (!img) {
        throw std::runtime_error("No frame was decoded");
        return false;
    }

    int width = img->d_w;
    int height = img->d_h;

    cv::Mat y(height, width, CV_8UC1, img->planes[0], img->stride[0]);
    cv::Mat u(height / 2, width / 2, CV_8UC1, img->planes[1], img->stride[1]);
    cv::Mat v(height / 2, width / 2, CV_8UC1, img->planes[2], img->stride[2]);

    cv::Mat u_resized, v_resized;
    cv::resize(u, u_resized, y.size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(v, v_resized, y.size(), 0, 0, cv::INTER_LINEAR);

    std::vector<cv::Mat> channels = {y, u_resized, v_resized};
    cv::Mat yuv, bgr;
    cv::merge(channels, yuv);
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR);
    frame = bgr.clone();
    aom_codec_destroy(&decoder);
    
    return true;
}


#pragma endregion
