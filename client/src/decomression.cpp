#include <decomression.h>
#include <zlib.h>
#include <cstring>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <ostream>

#pragma region common

void point(int num) { std::cout << "point " << num << std::endl; }

cv::Mat frameAddiiton(const cv::Mat& diff, const cv::Mat& old_frame) {
    cv::Mat frame;
    cv::add(old_frame, diff, frame);  // Складываем
    return frame;
}

void vector_add_chunk(const uchar* newData, const uchar* oldData, uchar* deltaData, size_t start, size_t end) {
    for (size_t i = start; i < end ; ++i){
        deltaData[i] = newData[i] + oldData[i] - 127;
    }
}

cv::Mat MatAdd (const cv::Mat& new_frame, const cv::Mat& old_frame) {
    cv::Mat delta = new_frame.clone();
    const size_t frameSize = new_frame.total() * new_frame.elemSize();

    // Определение количества потоков
    const size_t num_threads = std::min<size_t>(4, std::thread::hardware_concurrency());
    const size_t chunk_size = std::max<size_t>(1, frameSize / num_threads);
    // Векторы для хранения потоков
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    for (size_t i = 0; i < num_threads; ++i) {
        size_t start = i * chunk_size;
        size_t end = std::min(start + chunk_size, frameSize);

        if (start > frameSize) break;

        threads.emplace_back(vector_add_chunk, new_frame.data, old_frame.data, delta.data, start, end);
    }
    for (auto& t : threads)
        t.join();

    return delta;
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
    if (decompressedData.size() != originalSize) {
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

bool LZ4Decoder::lz4_decompress_dict() {
    // Устанавливаем словарь (если есть предыдущий блок)
    if (!prevBlock.empty()) 
        LZ4_setStreamDecode(decoder, prevBlock.data(), prevBlock.size());

    decompressedData.resize(originalSize);

    decompressedSize = LZ4_decompress_safe_continue(decoder, compressedData.data(), decompressedData.data(), compressedSize, originalSize);
    if (decompressedSize <= 0) {std::cerr << "Ошибка декомпрессии!" << std::endl; return false;}
    // Обновляем словарь: сохраняем последние 64 KB данных
    prevBlock.assign(decompressedData.data() + decompressedSize - std::min(decompressedSize, 64 * 1024), decompressedData.data() + decompressedSize);
    
    convertFromCleanDataChar();
    return decompressedSize > 0;
}

bool LZ4Decoder::lz4_decompress() {
    decompressedData.resize(originalSize);
    decompressedSize = LZ4_decompress_safe(compressedData.data(), decompressedData.data(), compressedSize, originalSize);
    if (decompressedSize <= 0) {
        std::cerr << "Ошибка декомпрессии!" << std::endl;
        return false;
    }
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

    if (inflateInit(&zStream) != Z_OK) std::cerr << "Error init zStream" << std::endl;

    zBuffer.resize(256 * 1024);
}

ZLIBDecoder::~ZLIBDecoder() { inflateEnd(&zStream); }

void ZLIBDecoder::convertFromCleanDataBytef() {
    if (decompressedSize == outputFrame.total() * outputFrame.elemSize())
        std::memcpy(outputFrame.data, decompressedData.data(), decompressedSize);
    else
        std::cerr << "uncorrect decompress data" << std::endl;
}

bool ZLIBDecoder::zlib_decompress_stream() {
    zStream.avail_in = compressedSize;
    zStream.next_in = compressedData.data();
    int ret;
    decompressedData.clear();

    // Сброс потока для новой декомпрессии
    if (inflateReset(&zStream) != Z_OK) {
        std::cerr << "Error reset inflate" << std::endl;
        return false;
    }

    zStream.avail_in = compressedSize;
    zStream.next_in = compressedData.data();
    do {
        zStream.avail_out = zBuffer.size();
        zStream.next_out = zBuffer.data();

        ret = inflate(&zStream, Z_NO_FLUSH);

        if (ret != Z_OK && ret != Z_STREAM_END) {
            std::cerr << "Error inflate: " << ret << std::endl;
            return false;
        }

        size_t bytesDecompressed = zBuffer.size() - zStream.avail_out;
        decompressedData.insert(decompressedData.end(), zBuffer.data(), zBuffer.data() + bytesDecompressed);

    } while (ret != Z_STREAM_END);

    decompressedSize = decompressedData.size();

    // Проверка и копирование данных в outputFrame
    size_t originalSize = outputFrame.total() * outputFrame.elemSize();
    if (decompressedSize == originalSize) {
        std::memcpy(outputFrame.data, decompressedData.data(), decompressedSize);
        return true;
    } else {
        std::cerr << "Uncorrect decompressed size. original" << originalSize << ", decompress " << decompressedSize << std::endl;
        return false;
    }
}

bool ZLIBDecoder::zlib_decompress() {
    decompressedData.resize(originalSize);
    uLongf decompressed_size = originalSize;

    int result = uncompress(decompressedData.data(), &decompressed_size, compressedData.data(), compressedData.size());
    if (result != Z_OK) {
        return false;  // Ошибка распаковки
    }

    if (outputFrame.total() * outputFrame.elemSize() == decompressedData.size())
        std::memcpy(outputFrame.data, decompressedData.data(), decompressedData.size());

    return outputFrame.total() * outputFrame.elemSize() == decompressedData.size();  // Успешная распаковка
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

#pragma region zstd

ZSTDDecoder::ZSTDDecoder() {
    // Создание контекста декомпрессии
    ZSTD_DCtx* dctx = ZSTD_createDCtx();
    if (!dctx) {
        throw std::runtime_error("Не удалось создать ZSTD_DCtx.");
    }
}
ZSTDDecoder::~ZSTDDecoder() {
    ZSTD_freeDCtx(dctx);
}

bool ZSTDDecoder::zstd_decompress_stream() {
    // Проверка входных данных
    if (compressedData.empty()) {
        throw std::runtime_error("Нет сжатых данных для декомпрессии.");
    }
    // Вычисление ожидаемого размера данных    
    decompressedData.clear();
    decompressedData.reserve(originalSize);

    // Буферы
    size_t const buff_in_size = ZSTD_DStreamInSize();
    size_t const buff_out_size = ZSTD_DStreamOutSize();
    std::vector<char> buff_in(buff_in_size);
    std::vector<char> buff_out(buff_out_size);

    // Подготовка входного буфера
    ZSTD_inBuffer in_buf = { compressedData.data(), compressedData.size(), 0 };

    while (in_buf.pos < in_buf.size) {
        ZSTD_outBuffer out_buf = { buff_out.data(), buff_out_size, 0 };

        size_t ret = ZSTD_decompressStream(dctx, &out_buf, &in_buf);
        if (ZSTD_isError(ret)) {
            throw std::runtime_error(ZSTD_getErrorName(ret));
        }

        decompressedData.insert(decompressedData.end(), buff_out.data(), buff_out.data() + out_buf.pos);
    }

    // Проверка длины
    if (outputFrame.total() * outputFrame.elemSize() == decompressedData.size())
        std::memcpy(outputFrame.data, decompressedData.data(), decompressedData.size());
    decompressedSize = decompressedData.size();
    return outputFrame.total() * outputFrame.elemSize() == decompressedData.size();  // Успешная распаковка
}

#pragma endregion