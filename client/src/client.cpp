
#include "client.h"
#include "decomression.h"

#include <opencv2/core/mat.hpp>
#include <thread>
#include <chrono>

#pragma region lz4

void lz4_concat_noprime(tcp::socket &socket) {
    int rows, cols, type;
    LZ4Decoder lz4Decoder = LZ4Decoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 1 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::high_resolution_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket,
                              boost::asio::buffer(&lz4Decoder.compressedSize, sizeof(lz4Decoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder.originalSize, sizeof(lz4Decoder.originalSize)));

            // Читаем данные кадра
            lz4Decoder.compressedData.resize(lz4Decoder.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder.compressedData));

            t1 = std::chrono::high_resolution_clock::now();  // После получения данных

            lz4Decoder.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder.lz4_decompress()) {
                t2 = std::chrono::high_resolution_clock::now();  // После сжатия
                cv::imshow("webcam", lz4Decoder.outputFrame);

                t3 = std::chrono::high_resolution_clock::now();  // После отображения

                std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                          << " decompress data: "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                          << " uncompressed data: " << lz4Decoder.decompressedSize
                          << " compressed data: " << lz4Decoder.compressedSize << " koef: "
                          << static_cast<double>(lz4Decoder.decompressedSize) /
                                 static_cast<double>(lz4Decoder.compressedSize)
                          << std::endl;
            }
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (...) {
        std::cerr << "Error in receiving frame. Closing..." << std::endl;
        cv::destroyAllWindows();
    }
}

void lz4_concat_prime(tcp::socket &socket) {
    int rows, cols, type;
    cv::Mat prevFrame;
    uint16_t currentFrame;
    LZ4Decoder lz4Decoder = LZ4Decoder();
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 2 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::high_resolution_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

            boost::asio::read(socket,
                              boost::asio::buffer(&lz4Decoder.compressedSize, sizeof(lz4Decoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder.originalSize, sizeof(lz4Decoder.originalSize)));

            // Читаем данные кадра
            lz4Decoder.compressedData.resize(lz4Decoder.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder.compressedData));

            t1 = std::chrono::high_resolution_clock::now();  // После получения данных

            lz4Decoder.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder.lz4_decompress()) {
                if ((currentFrame % 30) > 0 && !prevFrame.empty())
                    lz4Decoder.outputFrame = frameAddiiton(lz4Decoder.outputFrame, prevFrame);
                prevFrame = lz4Decoder.outputFrame.clone();
            }
            t2 = std::chrono::high_resolution_clock::now();  // После сжатия
            cv::imshow("webcam", lz4Decoder.outputFrame);

            t3 = std::chrono::high_resolution_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << lz4Decoder.decompressedSize
                      << " compressed data: " << lz4Decoder.compressedSize << " koef: "
                      << static_cast<double>(lz4Decoder.decompressedSize) /
                             static_cast<double>(lz4Decoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (...) {
        std::cerr << "Error in receiving frame. Closing..." << std::endl;
        cv::destroyAllWindows();
    }
}

void lz4_noconcat_noprime(tcp::socket &socket) {
    int rows, cols, type;
    LZ4Decoder lz4Decoder1, lz4Decoder2;

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 3 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::high_resolution_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket,
                              boost::asio::buffer(&lz4Decoder1.compressedSize, sizeof(lz4Decoder1.compressedSize)));

            boost::asio::read(socket,
                              boost::asio::buffer(&lz4Decoder2.compressedSize, sizeof(lz4Decoder2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder1.originalSize, sizeof(lz4Decoder1.originalSize)));
            lz4Decoder2.originalSize = lz4Decoder1.originalSize;

            // Читаем данные первого кадра
            lz4Decoder1.compressedData.resize(lz4Decoder1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder1.compressedData));

            // Читаем данные второго кадра
            lz4Decoder2.compressedData.resize(lz4Decoder2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder2.compressedData));

            t1 = std::chrono::high_resolution_clock::now();  // После получения данных

            lz4Decoder1.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder1.lz4_decompress()) cv::imshow("webcam1", lz4Decoder1.outputFrame);

            lz4Decoder2.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder2.lz4_decompress()) cv::imshow("webcam2", lz4Decoder2.outputFrame);
            t2 = std::chrono::high_resolution_clock::now();  // После сжатия

            t3 = std::chrono::high_resolution_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << lz4Decoder2.decompressedSize + lz4Decoder1.decompressedSize
                      << " compressed data: " << lz4Decoder1.compressedSize + lz4Decoder2.compressedSize << " koef: "
                      << static_cast<double>(lz4Decoder2.decompressedSize + lz4Decoder1.decompressedSize) /
                             static_cast<double>(lz4Decoder1.compressedSize + lz4Decoder2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (...) {
        std::cerr << "Error in receiving frame. Closing..." << std::endl;
        cv::destroyAllWindows();
    }
}

void lz4_noconcat_prime(tcp::socket &socket) {
    int rows, cols, type1, type2;
    cv::Mat prevFrame1, prevFrame2;
    LZ4Decoder lz4Decoder1, lz4Decoder2;

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 4 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::high_resolution_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type1, sizeof(type1)));
            boost::asio::read(socket, boost::asio::buffer(&type2, sizeof(type2)));

            boost::asio::read(socket,
                              boost::asio::buffer(&lz4Decoder1.compressedSize, sizeof(lz4Decoder1.compressedSize)));

            boost::asio::read(socket,
                              boost::asio::buffer(&lz4Decoder2.compressedSize, sizeof(lz4Decoder2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder1.originalSize, sizeof(lz4Decoder1.originalSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder2.originalSize, sizeof(lz4Decoder2.originalSize)));

            int currentFrame1;
            boost::asio::read(socket, boost::asio::buffer(&currentFrame1, sizeof(currentFrame1)));

            int currentFrame2;
            boost::asio::read(socket, boost::asio::buffer(&currentFrame2, sizeof(currentFrame2)));

            // Читаем данные первого кадра
            lz4Decoder1.compressedData.resize(lz4Decoder1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder1.compressedData));

            // Читаем данные второго кадра
            lz4Decoder2.compressedData.resize(lz4Decoder2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder2.compressedData));

            t1 = std::chrono::high_resolution_clock::now();  // После получения данных

            lz4Decoder1.outputFrame = cv::Mat(rows, cols, type1);
            if (lz4Decoder1.lz4_decompress()) {
                if ((currentFrame1 % 30) > 0 && !prevFrame1.empty())
                    lz4Decoder1.outputFrame = frameAddiiton(lz4Decoder1.outputFrame, prevFrame1);
                prevFrame1 = lz4Decoder1.outputFrame.clone();
                cv::imshow("webcam1", lz4Decoder1.outputFrame);
            }

            lz4Decoder2.outputFrame = cv::Mat(rows, cols, type2);
            if (lz4Decoder2.lz4_decompress()) {
                if ((currentFrame2 % 30) > 0 && !prevFrame2.empty())
                    lz4Decoder2.outputFrame = frameAddiiton(lz4Decoder2.outputFrame, prevFrame2);
                prevFrame2 = lz4Decoder2.outputFrame.clone();
                cv::imshow("webcam2", lz4Decoder2.outputFrame);
            }

            t2 = std::chrono::high_resolution_clock::now();  // После сжатия

            t3 = std::chrono::high_resolution_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << lz4Decoder1.decompressedSize + lz4Decoder2.decompressedSize
                      << " compressed data: " << lz4Decoder1.compressedSize + lz4Decoder1.compressedSize << " koef: "
                      << static_cast<double>(lz4Decoder1.decompressedSize + lz4Decoder2.decompressedSize) /
                             static_cast<double>(lz4Decoder1.compressedSize + lz4Decoder1.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (...) {
        std::cerr << "Error in receiving frame. Closing..." << std::endl;
        cv::destroyAllWindows();
    }
}

#pragma endregion

#pragma region zlib

void zlib_concat_noprime(tcp::socket &socket) {
    int rows, cols, type;

    // Декодер
    ZLIBDecoder zDecoder = ZLIBDecoder();
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 5 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::high_resolution_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder.compressedSize, sizeof(zDecoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder.originalSize, sizeof(zDecoder.originalSize)));

            // Читаем данные кадра
            zDecoder.compressedData.resize(zDecoder.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(zDecoder.compressedData));

            t1 = std::chrono::high_resolution_clock::now();  // После получения данных

            zDecoder.outputFrame = cv::Mat(rows, cols, type);
            if (zDecoder.zlib_decompress_stream()) {
                cv::imshow("webcam", zDecoder.outputFrame);
            }
            t2 = std::chrono::high_resolution_clock::now();  // После сжатия

            t3 = std::chrono::high_resolution_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << zDecoder.decompressedSize
                      << " compressed data: " << zDecoder.compressedSize << " koef: "
                      << static_cast<double>(zDecoder.decompressedSize) / static_cast<double>(zDecoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (...) {
        std::cerr << "Error in receiving frame. Closing..." << std::endl;
        cv::destroyAllWindows();
    }
}

void zlib_noconcat_noprime(tcp::socket &socket) {
    int rows, cols, type;
    cv::Mat frame1, frame2;
    std::vector<Bytef> compressed_data_frame_1, compressed_data_frame_2, uncompressed_data_frame_1,
        uncompressed_data_frame_2;
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 7 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::high_resolution_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            int compressed_size_frame_1;
            boost::asio::read(socket, boost::asio::buffer(&compressed_size_frame_1, sizeof(compressed_size_frame_1)));

            int compressed_size_frame_2;
            boost::asio::read(socket, boost::asio::buffer(&compressed_size_frame_2, sizeof(compressed_size_frame_2)));

            int uncompressed_size;
            boost::asio::read(socket, boost::asio::buffer(&uncompressed_size, sizeof(uncompressed_size)));

            // Читаем данные первого кадра
            compressed_data_frame_1.resize(compressed_size_frame_1);
            boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_1));

            // Читаем данные второго кадра
            compressed_data_frame_2.resize(compressed_size_frame_2);
            boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_2));

            t1 = std::chrono::high_resolution_clock::now();  // После получения данных

            frame1 = cv::Mat(rows, cols, type);
            if (zlib_decompress(compressed_data_frame_1, uncompressed_data_frame_1, uncompressed_size) == 0) {
                frame1 = convertFromCleanDataBytef(uncompressed_data_frame_1, frame1.rows, frame1.cols, frame1.type());
                cv::imshow("webcam1", frame1);
            }

            frame2 = cv::Mat(rows, cols, type);
            if (zlib_decompress(compressed_data_frame_2, uncompressed_data_frame_2, uncompressed_size) == 0) {
                frame2 = convertFromCleanDataBytef(uncompressed_data_frame_2, frame2.rows, frame2.cols, frame2.type());
                cv::imshow("webcam2", frame2);
            }
            t2 = std::chrono::high_resolution_clock::now();  // После сжатия

            t3 = std::chrono::high_resolution_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << uncompressed_size * 2
                      << " compressed data: " << compressed_size_frame_1 + compressed_size_frame_2 << " koef: "
                      << static_cast<double>(uncompressed_size * 2) /
                             static_cast<double>(compressed_size_frame_1 + compressed_size_frame_2)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (...) {
        std::cerr << "Error in receiving frame. Closing..." << std::endl;
        cv::destroyAllWindows();
    }
}
#pragma endregion

#pragma region aom

void aom_concat_noprime(tcp::socket &socket) {
    int rows, cols, type;
    cv::Mat frame;
    std::vector<uint8_t> compressed_data, uncompressed_data;
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 9 " << std::endl;

    try {
        while (true) {
            t0 = std::chrono::high_resolution_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            int compressed_size;
            boost::asio::read(socket, boost::asio::buffer(&compressed_size, sizeof(compressed_size)));

            // Читаем данные кадра
            compressed_data.resize(compressed_size);
            boost::asio::read(socket, boost::asio::buffer(compressed_data));

            t1 = std::chrono::high_resolution_clock::now();  // После получения данных

            frame = cv::Mat(rows, cols, type);
            if (!aom_decompress(compressed_data, frame)) {
                std::cout << "decompress error!!";
                continue;
            }
            t2 = std::chrono::high_resolution_clock::now();  // После сжатия
            cv::imshow("webcam", frame);

            t3 = std::chrono::high_resolution_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " compressed data: " << compressed_size << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (...) {
        std::cerr << "Error in receiving frame. Closing..." << std::endl;
        cv::destroyAllWindows();
    }
}

#pragma endregion
