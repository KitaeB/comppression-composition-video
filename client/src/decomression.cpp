#include <decomression.h> 

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
