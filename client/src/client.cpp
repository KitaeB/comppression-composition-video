
#include "client.h"
#include "decomression.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <chrono>
#include <opencv2/highgui.hpp>

#pragma region lz4

void lz4_concat_noprime(tcp::socket &socket) {
    int rows, cols, type;
    LZ4Decoder lz4Decoder = LZ4Decoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 1 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder.compressedSize, sizeof(lz4Decoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder.originalSize, sizeof(lz4Decoder.originalSize)));

            // Читаем данные кадра
            lz4Decoder.compressedData.resize(lz4Decoder.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            lz4Decoder.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder.lz4_decompress()) {
                t2 = std::chrono::steady_clock::now();  // После сжатия
                cv::imshow("webcam", lz4Decoder.outputFrame);

                t3 = std::chrono::steady_clock::now();  // После отображения

                std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                          << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                          << " uncompressed data: " << lz4Decoder.decompressedSize
                          << " compressed data: " << lz4Decoder.compressedSize << " koef: "
                          << static_cast<double>(lz4Decoder.decompressedSize) / static_cast<double>(lz4Decoder.compressedSize)
                          << std::endl;
            }
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
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
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder.compressedSize, sizeof(lz4Decoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder.originalSize, sizeof(lz4Decoder.originalSize)));

            // Читаем данные кадра
            lz4Decoder.compressedData.resize(lz4Decoder.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            lz4Decoder.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder.lz4_decompress()) {
                if ((currentFrame % 10) > 0 && !prevFrame.empty())
                    lz4Decoder.outputFrame = MatAdd(lz4Decoder.outputFrame, prevFrame);
                prevFrame = lz4Decoder.outputFrame.clone();
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия
            cv::imshow("webcam", lz4Decoder.outputFrame);

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << lz4Decoder.decompressedSize
                      << " compressed data: " << lz4Decoder.compressedSize << " koef: "
                      << static_cast<double>(lz4Decoder.decompressedSize) / static_cast<double>(lz4Decoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
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
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder1.compressedSize, sizeof(lz4Decoder1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder2.compressedSize, sizeof(lz4Decoder2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder1.originalSize, sizeof(lz4Decoder1.originalSize)));
            lz4Decoder2.originalSize = lz4Decoder1.originalSize;

            // Читаем данные первого кадра
            lz4Decoder1.compressedData.resize(lz4Decoder1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder1.compressedData));

            // Читаем данные второго кадра
            lz4Decoder2.compressedData.resize(lz4Decoder2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            lz4Decoder1.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder1.lz4_decompress()) cv::imshow("webcam1", lz4Decoder1.outputFrame);

            lz4Decoder2.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder2.lz4_decompress()) cv::imshow("webcam2", lz4Decoder2.outputFrame);
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

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
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void lz4_noconcat_prime(tcp::socket &socket) {
    int rows, cols, type, currentFrame1, currentFrame2;
    cv::Mat prevFrame1, prevFrame2, concat;
    LZ4Decoder lz4Decoder1, lz4Decoder2;

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 4 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&currentFrame1, sizeof(currentFrame1)));

            boost::asio::read(socket, boost::asio::buffer(&currentFrame2, sizeof(currentFrame2)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder1.compressedSize, sizeof(lz4Decoder1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder2.compressedSize, sizeof(lz4Decoder2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&lz4Decoder1.originalSize, sizeof(lz4Decoder1.originalSize)));
            lz4Decoder2.originalSize = lz4Decoder1.originalSize;

            // Читаем данные первого кадра
            lz4Decoder1.compressedData.resize(lz4Decoder1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder1.compressedData));

            // Читаем данные второго кадра
            lz4Decoder2.compressedData.resize(lz4Decoder2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(lz4Decoder2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            lz4Decoder1.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder1.lz4_decompress()) {
                if ((currentFrame1 % 10) > 0 && !prevFrame1.empty())
                    lz4Decoder1.outputFrame = MatAdd(prevFrame1, lz4Decoder1.outputFrame);
                prevFrame1 = lz4Decoder1.outputFrame.clone();
                cv::imshow("web1", lz4Decoder1.outputFrame);
            }

            lz4Decoder2.outputFrame = cv::Mat(rows, cols, type);
            if (lz4Decoder2.lz4_decompress()) {
                if ((currentFrame2 % 10) > 0 && !prevFrame2.empty())
                    lz4Decoder2.outputFrame = MatAdd(prevFrame2, lz4Decoder2.outputFrame);
                prevFrame2 = lz4Decoder2.outputFrame.clone();
                cv::imshow("web2", lz4Decoder2.outputFrame);
            }

            t2 = std::chrono::steady_clock::now();  // После сжатия


            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << lz4Decoder1.decompressedSize + lz4Decoder2.decompressedSize
                      << " compressed data: " << lz4Decoder1.compressedSize + lz4Decoder2.compressedSize << " koef: "
                      << static_cast<double>(lz4Decoder1.decompressedSize + lz4Decoder2.decompressedSize) /
                             static_cast<double>(lz4Decoder1.compressedSize + lz4Decoder2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
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
    point(5);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder.compressedSize, sizeof(zDecoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder.originalSize, sizeof(zDecoder.originalSize)));

            // Читаем данные кадра
            zDecoder.compressedData.resize(zDecoder.compressedSize);

            boost::asio::read(socket, boost::asio::buffer(zDecoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            zDecoder.outputFrame = cv::Mat(rows, cols, type);

            if (zDecoder.zlib_decompress_stream()) {
                cv::imshow("webcam", zDecoder.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << zDecoder.decompressedSize << " compressed data: " << zDecoder.compressedSize
                      << " koef: "
                      << static_cast<double>(zDecoder.decompressedSize) / static_cast<double>(zDecoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zlib_concat_prime(tcp::socket &socket) {
    int rows, cols, type, currentFrame;
    cv::Mat prevFrame;
    // Декодер
    ZLIBDecoder zDecoder = ZLIBDecoder();
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    point(6);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder.compressedSize, sizeof(zDecoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder.originalSize, sizeof(zDecoder.originalSize)));

            // Читаем данные кадра
            zDecoder.compressedData.resize(zDecoder.compressedSize);

            boost::asio::read(socket, boost::asio::buffer(zDecoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            zDecoder.outputFrame = cv::Mat(rows, cols, type);

            if (zDecoder.zlib_decompress_stream()) {
                if ((currentFrame % 10 == 0) && !prevFrame.empty())
                    zDecoder.outputFrame = MatAdd(zDecoder.outputFrame, prevFrame);
                prevFrame = zDecoder.outputFrame.clone();

                cv::imshow("webcam", zDecoder.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << zDecoder.decompressedSize << " compressed data: " << zDecoder.compressedSize
                      << " koef: "
                      << static_cast<double>(zDecoder.decompressedSize) / static_cast<double>(zDecoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zlib_noconcat_noprime(tcp::socket &socket) {
    int rows, cols, type;

    // Декодер
    ZLIBDecoder zDecoder_1 = ZLIBDecoder(), zDecoder_2 = ZLIBDecoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 7 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder_1.compressedSize, sizeof(zDecoder_1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder_2.compressedSize, sizeof(zDecoder_2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder_1.originalSize, sizeof(zDecoder_1.originalSize)));

            // Читаем данные первого кадра
            zDecoder_1.compressedData.resize(zDecoder_1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(zDecoder_1.compressedData));

            // Читаем данные второго кадра
            zDecoder_2.compressedData.resize(zDecoder_2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(zDecoder_2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            zDecoder_1.outputFrame = cv::Mat(rows, cols, type);
            if (zDecoder_1.zlib_decompress_stream()) {
                cv::imshow("webcam1", zDecoder_1.outputFrame);
            }

            zDecoder_2.outputFrame = cv::Mat(rows, cols, type);
            if (zDecoder_2.zlib_decompress_stream()) {
                cv::imshow("webcam2", zDecoder_2.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << zDecoder_1.decompressedSize + zDecoder_2.decompressedSize
                      << " compressed data: " << zDecoder_1.compressedSize + zDecoder_2.compressedSize << " koef: "
                      << static_cast<double>(zDecoder_1.decompressedSize + zDecoder_2.decompressedSize) /
                             static_cast<double>(zDecoder_1.compressedSize + zDecoder_2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zlib_noconcat_prime(tcp::socket &socket) {
    int rows, cols, type, currentFrame_1, currentFrame_2;
    cv::Mat prevFrame1, prevFrame2;

    // Декодер
    ZLIBDecoder zDecoder_1 = ZLIBDecoder(), zDecoder_2 = ZLIBDecoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 7 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&currentFrame_1, sizeof(currentFrame_1)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame_2, sizeof(currentFrame_2)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder_1.compressedSize, sizeof(zDecoder_1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder_2.compressedSize, sizeof(zDecoder_2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&zDecoder_1.originalSize, sizeof(zDecoder_1.originalSize)));
            zDecoder_2.originalSize = zDecoder_1.originalSize;

            // Читаем данные первого кадра
            zDecoder_1.compressedData.resize(zDecoder_1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(zDecoder_1.compressedData));

            // Читаем данные второго кадра
            zDecoder_2.compressedData.resize(zDecoder_2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(zDecoder_2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            zDecoder_1.outputFrame = cv::Mat(rows, cols, type);
            if (zDecoder_1.zlib_decompress_stream()) {
                if ((currentFrame_1 % 10 == 0) && !prevFrame1.empty())
                    zDecoder_1.outputFrame = MatAdd(zDecoder_1.outputFrame, prevFrame1);
                prevFrame1 = zDecoder_1.outputFrame.clone();

                cv::imshow("webcam1", zDecoder_1.outputFrame);
            }

            zDecoder_2.outputFrame = cv::Mat(rows, cols, type);
            if (zDecoder_2.zlib_decompress_stream()) {
                if ((currentFrame_2 % 10 == 0) && !prevFrame2.empty())
                    zDecoder_2.outputFrame = MatAdd(zDecoder_2.outputFrame, prevFrame2);
                prevFrame2 = zDecoder_2.outputFrame.clone();

                cv::imshow("webcam2", zDecoder_2.outputFrame);
            }

            t2 = std::chrono::steady_clock::now();  // После сжатия

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count()
                      << " uncompressed data: " << zDecoder_1.decompressedSize + zDecoder_2.decompressedSize
                      << " compressed data: " << zDecoder_1.compressedSize + zDecoder_2.compressedSize << " koef: "
                      << static_cast<double>(zDecoder_1.decompressedSize + zDecoder_2.decompressedSize) /
                             static_cast<double>(zDecoder_1.compressedSize + zDecoder_2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

#pragma endregion

#pragma region zstd

void zstd_concat_noprime(tcp::socket &socket) {
    int rows, cols, type;

    // Декодер
    ZSTDDecoder cDecoder = ZSTDDecoder();
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    point(9);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.compressedSize, sizeof(cDecoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.originalSize, sizeof(cDecoder.originalSize)));

            // Читаем данные кадра
            cDecoder.compressedData.resize(cDecoder.compressedSize);

            boost::asio::read(socket, boost::asio::buffer(cDecoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder.outputFrame = cv::Mat(rows, cols, type);

            if (cDecoder.zstd_decompress_stream()) {
                cv::imshow("webcam", cDecoder.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << cDecoder.decompressedSize << " compressed data: " << cDecoder.compressedSize
                      << " koef: "
                      << static_cast<double>(cDecoder.decompressedSize) / static_cast<double>(cDecoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zstd_concat_prime(tcp::socket &socket) {
    int rows, cols, type, currentFrame;
    cv::Mat prevFrame;
    // Декодер
    ZSTDDecoder cDecoder = ZSTDDecoder();
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    point(10);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.compressedSize, sizeof(cDecoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.originalSize, sizeof(cDecoder.originalSize)));

            // Читаем данные кадра
            cDecoder.compressedData.resize(cDecoder.compressedSize);

            boost::asio::read(socket, boost::asio::buffer(cDecoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder.outputFrame = cv::Mat(rows, cols, type);

            if (cDecoder.zstd_decompress_stream()) {
                if ((currentFrame % 10 == 0) && !prevFrame.empty())
                    cDecoder.outputFrame = MatAdd(cDecoder.outputFrame, prevFrame);
                prevFrame = cDecoder.outputFrame.clone();

                cv::imshow("webcam", cDecoder.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << cDecoder.decompressedSize << " compressed data: " << cDecoder.compressedSize
                      << " koef: "
                      << static_cast<double>(cDecoder.decompressedSize) / static_cast<double>(cDecoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zstd_noconcat_noprime(tcp::socket &socket) {
    int rows, cols, type;

    // Декодер
    ZSTDDecoder cDecoder_1 = ZSTDDecoder(), cDecoder_2 = ZSTDDecoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    point(11);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.compressedSize, sizeof(cDecoder_1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_2.compressedSize, sizeof(cDecoder_2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.originalSize, sizeof(cDecoder_1.originalSize)));

            // Читаем данные первого кадра
            cDecoder_1.compressedData.resize(cDecoder_1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_1.compressedData));

            // Читаем данные второго кадра
            cDecoder_2.compressedData.resize(cDecoder_2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder_1.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_1.zstd_decompress_stream()) {
                cv::imshow("webcam1", cDecoder_1.outputFrame);
            }

            cDecoder_2.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_2.zstd_decompress_stream()) {
                cv::imshow("webcam2", cDecoder_2.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << cDecoder_1.decompressedSize + cDecoder_2.decompressedSize
                      << " compressed data: " << cDecoder_1.compressedSize + cDecoder_2.compressedSize << " koef: "
                      << static_cast<double>(cDecoder_1.decompressedSize + cDecoder_2.decompressedSize) /
                             static_cast<double>(cDecoder_1.compressedSize + cDecoder_2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zstd_noconcat_prime(tcp::socket &socket) {
    int rows, cols, type, currentFrame_1, currentFrame_2;
    cv::Mat prevFrame1, prevFrame2;

    // Декодер
    ZSTDDecoder cDecoder_1 = ZSTDDecoder(), cDecoder_2 = ZSTDDecoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    point(12);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&currentFrame_1, sizeof(currentFrame_1)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame_2, sizeof(currentFrame_2)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.compressedSize, sizeof(cDecoder_1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_2.compressedSize, sizeof(cDecoder_2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.originalSize, sizeof(cDecoder_1.originalSize)));
            cDecoder_2.originalSize = cDecoder_1.originalSize;

            // Читаем данные первого кадра
            cDecoder_1.compressedData.resize(cDecoder_1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_1.compressedData));

            // Читаем данные второго кадра
            cDecoder_2.compressedData.resize(cDecoder_2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder_1.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_1.zstd_decompress_stream()) {
                if ((currentFrame_1 % 10 == 0) && !prevFrame1.empty())
                    cDecoder_1.outputFrame = MatAdd(cDecoder_1.outputFrame, prevFrame1);
                prevFrame1 = cDecoder_1.outputFrame.clone();

                cv::imshow("webcam1", cDecoder_1.outputFrame);
            }

            cDecoder_2.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_2.zstd_decompress_stream()) {
                if ((currentFrame_2 % 10 == 0) && !prevFrame2.empty())
                    cDecoder_2.outputFrame = MatAdd(cDecoder_2.outputFrame, prevFrame2);
                prevFrame2 = cDecoder_2.outputFrame.clone();

                cv::imshow("webcam2", cDecoder_2.outputFrame);
            }

            t2 = std::chrono::steady_clock::now();  // После сжатия

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count()
                      << " uncompressed data: " << cDecoder_1.decompressedSize + cDecoder_2.decompressedSize
                      << " compressed data: " << cDecoder_1.compressedSize + cDecoder_2.compressedSize << " koef: "
                      << static_cast<double>(cDecoder_1.decompressedSize + cDecoder_2.decompressedSize) /
                             static_cast<double>(cDecoder_1.compressedSize + cDecoder_2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

#pragma endregion

#pragma region zstd_gray

void zstd_gray_concat_noprime(tcp::socket &socket) {
    int rows, cols, type;

    // Декодер
    ZSTDDecoder cDecoder = ZSTDDecoder();
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    point(13);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.compressedSize, sizeof(cDecoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.originalSize, sizeof(cDecoder.originalSize)));

            // Читаем данные кадра
            cDecoder.compressedData.resize(cDecoder.compressedSize);

            boost::asio::read(socket, boost::asio::buffer(cDecoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder.outputFrame = cv::Mat(rows, cols, type);

            if (cDecoder.zstd_decompress_stream()) {
                cv::imshow("webcam", cDecoder.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << cDecoder.decompressedSize << " compressed data: " << cDecoder.compressedSize
                      << " koef: "
                      << static_cast<double>(cDecoder.decompressedSize) / static_cast<double>(cDecoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zstd_gray_concat_prime(tcp::socket &socket) {
    int rows, cols, type, currentFrame;
    cv::Mat prevFrame;
    // Декодер
    ZSTDDecoder cDecoder = ZSTDDecoder();
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    point(14);
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.compressedSize, sizeof(cDecoder.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder.originalSize, sizeof(cDecoder.originalSize)));

            // Читаем данные кадра
            cDecoder.compressedData.resize(cDecoder.compressedSize);

            boost::asio::read(socket, boost::asio::buffer(cDecoder.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder.outputFrame = cv::Mat(rows, cols, type);

            if (cDecoder.zstd_decompress_stream()) {
                if ((currentFrame % 10 == 0) && !prevFrame.empty())
                    cDecoder.outputFrame = MatAdd(cDecoder.outputFrame, prevFrame);
                prevFrame = cDecoder.outputFrame.clone();

                cv::imshow("webcam", cDecoder.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << cDecoder.decompressedSize << " compressed data: " << cDecoder.compressedSize
                      << " koef: "
                      << static_cast<double>(cDecoder.decompressedSize) / static_cast<double>(cDecoder.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zstd_gray_noconcat_noprime(tcp::socket &socket) {
    int rows, cols, type;

    // Декодер
    ZSTDDecoder cDecoder_1 = ZSTDDecoder(), cDecoder_2 = ZSTDDecoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 15 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.compressedSize, sizeof(cDecoder_1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_2.compressedSize, sizeof(cDecoder_2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.originalSize, sizeof(cDecoder_1.originalSize)));

            // Читаем данные первого кадра
            cDecoder_1.compressedData.resize(cDecoder_1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_1.compressedData));

            // Читаем данные второго кадра
            cDecoder_2.compressedData.resize(cDecoder_2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder_1.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_1.zstd_decompress_stream()) {
                cv::imshow("webcam1", cDecoder_1.outputFrame);
            }

            cDecoder_2.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_2.zstd_decompress_stream()) {
                cv::imshow("webcam2", cDecoder_2.outputFrame);
            }
            t2 = std::chrono::steady_clock::now();  // После сжатия

            t3 = std::chrono::steady_clock::now();  // После отображения

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                      << " uncompressed data: " << cDecoder_1.decompressedSize + cDecoder_2.decompressedSize
                      << " compressed data: " << cDecoder_1.compressedSize + cDecoder_2.compressedSize << " koef: "
                      << static_cast<double>(cDecoder_1.decompressedSize + cDecoder_2.decompressedSize) /
                             static_cast<double>(cDecoder_1.compressedSize + cDecoder_2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

void zstd_gray_noconcat_prime(tcp::socket &socket) {
    int rows, cols, type, currentFrame_1, currentFrame_2;
    cv::Mat prevFrame1, prevFrame2;

    // Декодер
    ZSTDDecoder cDecoder_1 = ZSTDDecoder(), cDecoder_2 = ZSTDDecoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    std::cout << "point 16 " << std::endl;
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения данных
            // Читаем метаданные
            boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
            boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
            boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

            boost::asio::read(socket, boost::asio::buffer(&currentFrame_1, sizeof(currentFrame_1)));
            boost::asio::read(socket, boost::asio::buffer(&currentFrame_2, sizeof(currentFrame_2)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.compressedSize, sizeof(cDecoder_1.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_2.compressedSize, sizeof(cDecoder_2.compressedSize)));

            boost::asio::read(socket, boost::asio::buffer(&cDecoder_1.originalSize, sizeof(cDecoder_1.originalSize)));
            cDecoder_2.originalSize = cDecoder_1.originalSize;

            // Читаем данные первого кадра
            cDecoder_1.compressedData.resize(cDecoder_1.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_1.compressedData));

            // Читаем данные второго кадра
            cDecoder_2.compressedData.resize(cDecoder_2.compressedSize);
            boost::asio::read(socket, boost::asio::buffer(cDecoder_2.compressedData));

            t1 = std::chrono::steady_clock::now();  // После получения данных

            cDecoder_1.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_1.zstd_decompress_stream()) {
                if ((currentFrame_1 % 10 == 0) && !prevFrame1.empty())
                    cDecoder_1.outputFrame = MatAdd(cDecoder_1.outputFrame, prevFrame1);
                prevFrame1 = cDecoder_1.outputFrame.clone();

                cv::imshow("webcam1", cDecoder_1.outputFrame);
            }

            cDecoder_2.outputFrame = cv::Mat(rows, cols, type);
            if (cDecoder_2.zstd_decompress_stream()) {
                if ((currentFrame_2 % 10 == 0) && !prevFrame2.empty())
                    cDecoder_2.outputFrame = MatAdd(cDecoder_2.outputFrame, prevFrame2);
                prevFrame2 = cDecoder_2.outputFrame.clone();

                cv::imshow("webcam2", cDecoder_2.outputFrame);
            }

            t2 = std::chrono::steady_clock::now();  // После сжатия

            std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                      << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t2).count()
                      << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count()
                      << " uncompressed data: " << cDecoder_1.decompressedSize + cDecoder_2.decompressedSize
                      << " compressed data: " << cDecoder_1.compressedSize + cDecoder_2.compressedSize << " koef: "
                      << static_cast<double>(cDecoder_1.decompressedSize + cDecoder_2.decompressedSize) /
                             static_cast<double>(cDecoder_1.compressedSize + cDecoder_2.compressedSize)
                      << std::endl;
            // Обязательно waitKey
            if (cv::waitKey(1) == 27) {  // Нажал ESC
                cv::destroyAllWindows();
                break;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in receiving frame. Closing..." << ex.what() << std::endl;
        cv::destroyAllWindows();
    }
}

#pragma endregion
