#include <aom/aom_codec.h>

#include <chrono>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <ostream>
#include <thread>

#include "compression.h"
#include "server.h"

#pragma region tcp_server

TcpServer::TcpServer(const uint16_t port)
    : socket(io_context), acceptor(io_context, tcp::endpoint(tcp::v4(), port)), PORT(port) {}

void TcpServer::NewConnect() {
    // Закрываем старое соединение, если есть
    if (socket.is_open()) {
        boost::system::error_code ec;
        socket.close();
        if (ec) {
            std::cerr << "Error connection close" << std::endl;
        }
    }

    // Открываем новое соединение
    std::cout << "Waiting new connect.." << std::endl;
    acceptor.accept(socket);
    if (!socket.is_open()) {
        std::cerr << "failed to open socket" << std::endl;
    }
}

tcp::socket &TcpServer::GetSocket() { return socket; }
bool TcpServer::TestConnect() { return socket.is_open(); }

#pragma endregion

#pragma region common

// Функция захвата кадров для отдельной камеры
void captureFrames(CameraState &camState, int camIndex) {
    camState.cap.set(cv::CAP_PROP_FRAME_WIDTH, camState.width);
    camState.cap.set(cv::CAP_PROP_FRAME_HEIGHT, camState.height);
    camState.cap.set(cv::CAP_PROP_FPS, 30);
    if (!camState.cap.isOpened()) {
        std::cerr << "Failed to open camera " << camIndex << std::endl;
        camState.running = false;
        return;
    }

    cv::Mat frame;
    while (camState.running) {
        if (camState.cap.read(frame) && !frame.empty()) {
            std::lock_guard<std::mutex> lock(camState.frameMutex);

            // Добавление гаус фильтра 3x3
            if (camState.useFiltre)
                cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);

            camState.lastFrame = frame.clone();
            camState.frameReady = true;

            // Добавим определение времени захвата и на основании этого сможем частоту камеры получить.
            camState.timeToFrame = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - camState.t).count();
            camState.t = std::chrono::steady_clock::now();

        } else if (!camState.cap.read(frame) && !camState.file.empty()) {   
            camState.cap.open(camState.file);
        } else {
            std::cerr << "Failed to capture frame from camera/video " << camIndex << std::endl;
        }
        // Небольшая задержка, чтобы не перегружать CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

#pragma endregion

#pragma region lz4

void lz4_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame;
    LZ4Coder lz4Coder = LZ4Coder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // После получения
            t1 = std::chrono::steady_clock::now();

            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            if (last_duration > 20 && lz4Coder.acceleration < 100)
                lz4Coder.acceleration++;
            else if (last_duration < 15 && lz4Coder.acceleration > 1)
                lz4Coder.acceleration--;
            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение

            // Сожмём данные с lz4-fast
            if (lz4Coder.lz4_compress_fast(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " <<
                // type
                // << std::endl;

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                boost::asio::write(socket, boost::asio::buffer(&lz4Coder.compressedSize, sizeof(lz4Coder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&lz4Coder.uncompressedSize, sizeof(lz4Coder.uncompressedSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(lz4Coder.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " acceleration: " << lz4Coder.acceleration << " uncompressed data: " << lz4Coder.uncompressedSize
                          << " compressed data: " << lz4Coder.compressedSize << " koef: "
                          << static_cast<double>(lz4Coder.uncompressedSize) / static_cast<double>(lz4Coder.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void lz4_concat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame, prevFrame, tempFrame;
    uint16_t currentFrame;
    LZ4Coder lz4Coder = LZ4Coder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // После получения
            t1 = std::chrono::steady_clock::now();

            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            // cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);
            tempFrame = frame.clone();

            if ((currentFrame % 30) > 0 && !prevFrame.empty()) {
                frame = frameSubstraction(frame, prevFrame);  // Производим вычитания
            }

            prevFrame = tempFrame.clone();
            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            if (last_duration > 20 && lz4Coder.acceleration < 200)
                lz4Coder.acceleration++;
            else if (last_duration < 15 && lz4Coder.acceleration > 1)
                lz4Coder.acceleration--;

            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение

            // Сожмём данные с zlib-default
            if (lz4Coder.lz4_compress_fast(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));
                boost::asio::write(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

                boost::asio::write(socket, boost::asio::buffer(&lz4Coder.compressedSize, sizeof(lz4Coder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&lz4Coder.uncompressedSize, sizeof(lz4Coder.uncompressedSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(lz4Coder.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " acceleration: " << lz4Coder.acceleration << " currentFrame: " << currentFrame
                          << " uncompressed data: " << lz4Coder.uncompressedSize
                          << " compressed data: " << lz4Coder.compressedSize << " koef: "
                          << static_cast<double>(lz4Coder.uncompressedSize) / static_cast<double>(lz4Coder.compressedSize)
                          << std::endl;
                currentFrame++;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void lz4_noconcat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame, prevFrame;
    LZ4Coder lz4Coder1 = LZ4Coder(), lz4Coder2 = LZ4Coder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            if (last_duration > 20 && lz4Coder1.acceleration < 100)
                lz4Coder1.acceleration++;
            else if (last_duration < 15 && lz4Coder1.acceleration > 1)
                lz4Coder1.acceleration--;
            lz4Coder2.acceleration = lz4Coder1.acceleration;

            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение

            // Сожмём данные с zlib-default
            if ((lz4Coder1.lz4_compress_fast(frame1) > 0) && (lz4Coder2.lz4_compress_fast(frame2) > 0)) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type = frame1.type();

                // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " <<
                // type
                // << std::endl;

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&lz4Coder1.compressedSize, sizeof(lz4Coder1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&lz4Coder2.compressedSize, sizeof(lz4Coder2.compressedSize)));

                // Мы считаем, что исходные данные одинаковые по размеры
                boost::asio::write(socket, boost::asio::buffer(&lz4Coder1.uncompressedSize, sizeof(lz4Coder1.uncompressedSize)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(lz4Coder1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(lz4Coder2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " acceleration: " << lz4Coder1.acceleration
                          << " uncompressed data: " << lz4Coder1.uncompressedSize * 2
                          << " compressed data: " << lz4Coder1.compressedSize + lz4Coder2.compressedSize << " koef: "
                          << static_cast<double>(lz4Coder1.uncompressedSize * 2) /
                                 static_cast<double>(lz4Coder1.compressedSize + lz4Coder2.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void lz4_noconcat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, prevFrame1, prevFrame2, tempFrame1, tempFrame2;
    LZ4Coder lz4Coder1, lz4Coder2;

    LZ4_stream_t *lz4Stream = LZ4_createStream();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                    cam1.currentFrame++;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                    cam2.currentFrame++;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            if (last_duration > 20 && lz4Coder1.acceleration < 100)
                lz4Coder1.acceleration++;
            else if (last_duration < 15 && lz4Coder1.acceleration > 1)
                lz4Coder1.acceleration--;
            lz4Coder2.acceleration = lz4Coder1.acceleration;

            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение

            tempFrame1 = frame1.clone();
            // cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);
            if ((cam1.currentFrame % 30) > 0 && !prevFrame1.empty()) {
                frame1 = frameSubstraction(frame1, prevFrame1);  // Производим
                                                                 // вычитания
            }
            prevFrame1 = tempFrame1.clone();

            tempFrame2 = frame2.clone();
            if ((cam2.currentFrame % 30) > 0 && !prevFrame2.empty()) {
                frame2 = frameSubstraction(frame2, prevFrame2);  // Производим
                                                                 // вычитания
            }
            prevFrame2 = tempFrame2.clone();

            // Сожмём данные с zlib-default
            if ((lz4Coder1.lz4_compress_fast(frame1) > 0) && (lz4Coder2.lz4_compress_fast(frame2) > 0)) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type1 = frame1.type();
                int type2 = frame2.type();

                // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " <<
                // type
                // << std::endl;

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type1, sizeof(type1)));
                boost::asio::write(socket, boost::asio::buffer(&type2, sizeof(type2)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&lz4Coder1.compressedSize, sizeof(lz4Coder1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&lz4Coder2.compressedSize, sizeof(lz4Coder2.compressedSize)));

                // Мы не можем считать, что исходные данные одинаковые по размеру,
                // поскольку у нас один кадр может быть ключевым, а другой нет
                boost::asio::write(socket, boost::asio::buffer(&lz4Coder1.uncompressedSize, sizeof(lz4Coder1.uncompressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&lz4Coder2.uncompressedSize, sizeof(lz4Coder2.uncompressedSize)));

                // Также надо отправить данные о том у нас кадр ключеввой или нет
                boost::asio::write(socket, boost::asio::buffer(&cam1.currentFrame, sizeof(cam1.currentFrame)));
                boost::asio::write(socket, boost::asio::buffer(&cam2.currentFrame, sizeof(cam2.currentFrame)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(lz4Coder1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(lz4Coder2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " acceleration: " << lz4Coder1.acceleration
                          << " uncompressed data: " << lz4Coder1.uncompressedSize + lz4Coder2.uncompressedSize
                          << " compressed data: " << lz4Coder2.compressedSize + lz4Coder1.compressedSize << " koef: "
                          << static_cast<double>(lz4Coder1.uncompressedSize + lz4Coder2.uncompressedSize) /
                                 static_cast<double>(lz4Coder2.compressedSize + lz4Coder1.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

#pragma endregion

#pragma region zlib

void zlib_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }
    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame;

    // Кодер
    ZLIBCoder zCoder = ZLIBCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    // основной цикл
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // После получения
            t1 = std::chrono::steady_clock::now();
            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            t2 = std::chrono::steady_clock::now();  // Изменение размера и объединение

            // Сожмём данные с zlib-default
            if (zCoder.zlib_compress_stream(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                boost::asio::write(socket, boost::asio::buffer(&zCoder.compressedSize, sizeof(zCoder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&zCoder.originalSize, sizeof(zCoder.originalSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(zCoder.compressedData));
                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << zCoder.originalSize << " compressed data: " << zCoder.compressedSize
                          << " koef: " << static_cast<double>(zCoder.originalSize) / static_cast<double>(zCoder.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zlib_concat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }
    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame, prevFrame, tempFrame;
    int currentFrame = 0;
    // Кодер
    ZLIBCoder zCoder = ZLIBCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    // основной цикл
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // После получения
            t1 = std::chrono::steady_clock::now();

            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            t2 = std::chrono::steady_clock::now();  // Изменение размера и объединение

            tempFrame = frame.clone();
            if ((currentFrame % 30) == 0 && !zCoder.inputFrame.empty()) frame = frameSubstraction(frame, prevFrame);
            prevFrame = tempFrame.clone();

            // Сожмём данные с zlib-default
            if (zCoder.zlib_compress_stream(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));
                boost::asio::write(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

                boost::asio::write(socket, boost::asio::buffer(&zCoder.compressedSize, sizeof(zCoder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&zCoder.originalSize, sizeof(zCoder.originalSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(zCoder.compressedData));
                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << zCoder.originalSize << " compressed data: " << zCoder.compressedSize
                          << " koef: " << static_cast<double>(zCoder.originalSize) / static_cast<double>(zCoder.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zlib_noconcat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, prevFrame;
    // Кодеры
    ZLIBCoder zCoder_1 = ZLIBCoder(), zCoder_2 = ZLIBCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение
            // Сожмём данные с zlib-fast
            if ((zCoder_1.zlib_compress_stream(frame1) > 0) && (zCoder_2.zlib_compress_stream(frame2)) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type = frame1.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&zCoder_1.compressedSize, sizeof(zCoder_1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&zCoder_2.compressedSize, sizeof(zCoder_2.compressedSize)));

                // Мы считаем, что исходные данные одинаковые по размеры
                boost::asio::write(socket, boost::asio::buffer(&zCoder_1.originalSize, sizeof(zCoder_1.originalSize)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(zCoder_1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(zCoder_2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << zCoder_1.originalSize * 2
                          << " compressed data: " << zCoder_1.compressedSize + zCoder_2.compressedSize << " koef: "
                          << static_cast<double>(zCoder_1.originalSize * 2) /
                                 static_cast<double>(zCoder_1.compressedSize + zCoder_2.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zlib_noconcat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, prevFrame_1, prevFrame_2, tempFrame_1, tempFrame_2;
    // Кодеры
    ZLIBCoder zCoder_1 = ZLIBCoder(), zCoder_2 = ZLIBCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение
            tempFrame_1 = frame1.clone();
            if ((cam1.currentFrame % 30) == 0 && !prevFrame_1.empty()) frame1 = frameSubstraction(frame1, prevFrame_1);
            prevFrame_1 = tempFrame_1.clone();

            tempFrame_2 = frame2.clone();
            if ((cam2.currentFrame % 30) == 0 && !prevFrame_2.empty()) frame2 = frameSubstraction(frame2, prevFrame_2);
            prevFrame_2 = tempFrame_2.clone();

            // Сожмём данные с zlib-fast
            if ((zCoder_1.zlib_compress_stream(frame1) > 0) && (zCoder_2.zlib_compress_stream(frame2)) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type1 = frame1.type();
                int type2 = frame2.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type1, sizeof(type1)));
                boost::asio::write(socket, boost::asio::buffer(&type2, sizeof(type2)));

                boost::asio::write(socket, boost::asio::buffer(&cam1.currentFrame, sizeof(cam1.currentFrame)));
                boost::asio::write(socket, boost::asio::buffer(&cam2.currentFrame, sizeof(cam2.currentFrame)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&zCoder_1.compressedSize, sizeof(zCoder_1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&zCoder_2.compressedSize, sizeof(zCoder_2.compressedSize)));

                // Мы считаем, что исходные данные могут быть разные по размеру
                boost::asio::write(socket, boost::asio::buffer(&zCoder_1.originalSize, sizeof(zCoder_1.originalSize)));
                boost::asio::write(socket, boost::asio::buffer(&zCoder_2.originalSize, sizeof(zCoder_2.originalSize)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(zCoder_1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(zCoder_2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << zCoder_1.originalSize + zCoder_2.originalSize
                          << " compressed data: " << zCoder_1.compressedSize + zCoder_2.compressedSize << " koef: "
                          << static_cast<double>(zCoder_1.originalSize + zCoder_2.originalSize) /
                                 static_cast<double>(zCoder_1.compressedSize + zCoder_2.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}
#pragma endregion

#pragma region zstd

void zstd_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }
    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame;

    // Кодер
    ZSTDCoder cCoder = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    // основной цикл
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // После получения
            t1 = std::chrono::steady_clock::now();
            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            t2 = std::chrono::steady_clock::now();  // Изменение размера и объединение

            // Сожмём данные с zlib-default
            if (cCoder.zstd_compress(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.compressedSize, sizeof(cCoder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.originalSize, sizeof(cCoder.originalSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(cCoder.compressedData));
                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder.originalSize << " compressed data: " << cCoder.compressedSize
                          << " koef: " << static_cast<double>(cCoder.originalSize) / static_cast<double>(cCoder.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zstd_concat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }
    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame, prevFrame, tempFrame;
    int currentFrame = 0;
    // Кодер
    ZSTDCoder cCoder = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    // основной цикл
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // После получения
            t1 = std::chrono::steady_clock::now();
            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            t2 = std::chrono::steady_clock::now();  // Изменение размера и объединение

            tempFrame = frame.clone();
            if ((currentFrame % 30) == 0 && !cCoder.inputFrame.empty()) frame = frameSubstraction(frame, prevFrame);
            prevFrame = tempFrame.clone();

            // Сожмём данные с zlib-default
            if (cCoder.zstd_compress(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));
                boost::asio::write(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.compressedSize, sizeof(cCoder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.originalSize, sizeof(cCoder.originalSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(cCoder.compressedData));
                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder.originalSize << " compressed data: " << cCoder.compressedSize
                          << " koef: " << static_cast<double>(cCoder.originalSize) / static_cast<double>(cCoder.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zstd_noconcat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, prevFrame;
    // Кодеры
    ZSTDCoder cCoder_1 = ZSTDCoder(), cCoder_2 = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение
            // Сожмём данные с zlib-fast
            if ((cCoder_1.zstd_compress(frame1) > 0) && (cCoder_2.zstd_compress(frame2)) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type = frame1.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.compressedSize, sizeof(cCoder_1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_2.compressedSize, sizeof(cCoder_2.compressedSize)));

                // Мы считаем, что исходные данные одинаковые по размеры
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.originalSize, sizeof(cCoder_1.originalSize)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(cCoder_1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(cCoder_2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder_1.originalSize * 2
                          << " compressed data: " << cCoder_1.compressedSize + cCoder_2.compressedSize << " koef: "
                          << static_cast<double>(cCoder_1.originalSize * 2) /
                                 static_cast<double>(cCoder_1.compressedSize + cCoder_2.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zstd_noconcat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, prevFrame_1, prevFrame_2, tempFrame_1, tempFrame_2;
    // Кодеры
    ZSTDCoder cCoder_1 = ZSTDCoder(), cCoder_2 = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            t2 = std::chrono::steady_clock::now();  // Изменение размера и
                                                    // объединение
            tempFrame_1 = frame1.clone();
            if ((cam1.currentFrame % 30) == 0 && !prevFrame_1.empty()) frame1 = frameSubstraction(frame1, prevFrame_1);
            prevFrame_1 = tempFrame_1.clone();

            tempFrame_2 = frame2.clone();
            if ((cam2.currentFrame % 30) == 0 && !prevFrame_2.empty()) frame2 = frameSubstraction(frame2, prevFrame_2);
            prevFrame_2 = tempFrame_2.clone();

            // Сожмём данные с zlib-fast
            if ((cCoder_1.zstd_compress(frame1) > 0) && (cCoder_2.zstd_compress(frame2)) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type1 = frame1.type();
                int type2 = frame2.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type1, sizeof(type1)));
                boost::asio::write(socket, boost::asio::buffer(&type2, sizeof(type2)));

                boost::asio::write(socket, boost::asio::buffer(&cam1.currentFrame, sizeof(cam1.currentFrame)));
                boost::asio::write(socket, boost::asio::buffer(&cam2.currentFrame, sizeof(cam2.currentFrame)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.compressedSize, sizeof(cCoder_1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_2.compressedSize, sizeof(cCoder_2.compressedSize)));

                // Мы считаем, что исходные данные могут быть разные по размеру
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.originalSize, sizeof(cCoder_1.originalSize)));
                boost::asio::write(socket, boost::asio::buffer(&cCoder_2.originalSize, sizeof(cCoder_2.originalSize)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(cCoder_1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(cCoder_2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder_1.originalSize + cCoder_2.originalSize
                          << " compressed data: " << cCoder_1.compressedSize + cCoder_2.compressedSize << " koef: "
                          << static_cast<double>(cCoder_1.originalSize + cCoder_2.originalSize) /
                                 static_cast<double>(cCoder_1.compressedSize + cCoder_2.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

#pragma endregion

#pragma region zstd_gray

void zstd_gray_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }
    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame;

    // Кодер
    ZSTDCoder cCoder = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    // основной цикл
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // После получения
            t1 = std::chrono::steady_clock::now();
            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            if (frame.channels() == 3)
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            t2 = std::chrono::steady_clock::now();  // Изменение размера и объединение

            // Сожмём данные с zlib-default
            if (cCoder.zstd_compress(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.compressedSize, sizeof(cCoder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.originalSize, sizeof(cCoder.originalSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(cCoder.compressedData));
                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder.originalSize << " compressed data: " << cCoder.compressedSize
                          << " koef: " << static_cast<double>(cCoder.originalSize) / static_cast<double>(cCoder.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zstd_gray_concat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }
    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame, prevFrame, tempFrame;
    int currentFrame = 0;
    // Кодер
    ZSTDCoder cCoder = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    // основной цикл
    try {
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // После получения
            t1 = std::chrono::steady_clock::now();
            // производим соединение кадров
            cv::resize(frame2, frame2, frame1.size());
            cv::hconcat(frame1, frame2, frame);

            if (frame.channels() == 3)
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

            auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

            t2 = std::chrono::steady_clock::now();  // Изменение размера и объединение

            tempFrame = frame.clone();
            if ((currentFrame % 30) == 0 && !cCoder.inputFrame.empty()) frame = frameSubstraction(frame, prevFrame);
            prevFrame = tempFrame.clone();

            // Сожмём данные с zlib-default
            if (cCoder.zstd_compress(frame) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame.rows;
                int cols = frame.cols;
                int type = frame.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));
                boost::asio::write(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.compressedSize, sizeof(cCoder.compressedSize)));

                boost::asio::write(socket, boost::asio::buffer(&cCoder.originalSize, sizeof(cCoder.originalSize)));

                // Отправка данных клиенту
                boost::asio::write(socket, boost::asio::buffer(cCoder.compressedData));
                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder.originalSize << " compressed data: " << cCoder.compressedSize
                          << " koef: " << static_cast<double>(cCoder.originalSize) / static_cast<double>(cCoder.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zstd_gray_noconcat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, prevFrame;
    // Кодеры
    ZSTDCoder cCoder_1 = ZSTDCoder(), cCoder_2 = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            t2 = std::chrono::steady_clock::now();             // Изменение размера и объединение
            if (frame1.channels() == 3)
                cv::cvtColor(frame1, frame1, cv::COLOR_BGR2GRAY);
            if (frame2.channels() == 3)
                cv::cvtColor(frame2, frame2, cv::COLOR_BGR2GRAY);

            // Сожмём данные с zlib-fast
            if ((cCoder_1.zstd_compress(frame1) > 0) && (cCoder_2.zstd_compress(frame2)) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type = frame1.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.compressedSize, sizeof(cCoder_1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_2.compressedSize, sizeof(cCoder_2.compressedSize)));

                // Мы считаем, что исходные данные одинаковые по размеры
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.originalSize, sizeof(cCoder_1.originalSize)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(cCoder_1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(cCoder_2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder_1.originalSize * 2
                          << " compressed data: " << cCoder_1.compressedSize + cCoder_2.compressedSize << " koef: "
                          << static_cast<double>(cCoder_1.originalSize * 2) /
                                 static_cast<double>(cCoder_1.compressedSize + cCoder_2.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

void zstd_gray_noconcat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2) {
    // Проверяем, открылась ли хотя бы одна камера
    if (!cam1.cap.isOpened() && !cam2.cap.isOpened()) {
        std::cerr << "No cameras opened" << std::endl;
        return;
    }

    // Запускаем потоки для захвата кадров
    std::thread cam1Thread(captureFrames, std::ref(cam1), 0);
    std::thread cam2Thread(captureFrames, std::ref(cam2), 1);

    // Объявляем фреймы
    cv::Mat frame1, frame2, prevFrame_1, prevFrame_2, tempFrame_1, tempFrame_2;
    // Кодеры
    ZSTDCoder cCoder_1 = ZSTDCoder(), cCoder_2 = ZSTDCoder();

    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    try {
        // основной цикл
        while (true) {
            t0 = std::chrono::steady_clock::now();  // До получения картинки
            // Читаем данные с камер
            bool hasNewFrame = false;
            {
                std::lock_guard<std::mutex> lock1(cam1.frameMutex);
                if (cam1.frameReady && !cam1.lastFrame.empty()) {
                    frame1 = cam1.lastFrame.clone();
                    cam1.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame1.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame1 = cv::Mat(cam1.height, cam1.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }
            {
                std::lock_guard<std::mutex> lock2(cam2.frameMutex);
                if (cam2.frameReady && !cam2.lastFrame.empty()) {
                    frame2 = cam2.lastFrame.clone();
                    cam2.frameReady = false;
                    hasNewFrame = true;
                } else if (!frame2.empty()) {
                    // Используем предыдущий кадр
                } else {
                    frame2 = cv::Mat(cam2.height, cam2.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }
            }

            if (!hasNewFrame) {
                // Если нет новых кадров, ждем немного
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            t2 = std::chrono::steady_clock::now();  // Изменение размера и объединение
            if (frame1.channels() == 3)
                cv::cvtColor(frame1, frame1, cv::COLOR_BGR2GRAY);
            if (frame2.channels() == 3)
                cv::cvtColor(frame2, frame2, cv::COLOR_BGR2GRAY);

            tempFrame_1 = frame1.clone();
            if ((cam1.currentFrame % 30) == 0 && !prevFrame_1.empty()) frame1 = frameSubstraction(frame1, prevFrame_1);
            prevFrame_1 = tempFrame_1.clone();

            tempFrame_2 = frame2.clone();
            if ((cam2.currentFrame % 30) == 0 && !prevFrame_2.empty()) frame2 = frameSubstraction(frame2, prevFrame_2);
            prevFrame_2 = tempFrame_2.clone();

            // Сожмём данные с zlib-fast
            if ((cCoder_1.zstd_compress(frame1) > 0) && (cCoder_2.zstd_compress(frame2)) > 0) {
                t3 = std::chrono::steady_clock::now();  // После сжатия
                // Объявим метаданные передаваеммого кадра
                int rows = frame1.rows;
                int cols = frame1.cols;
                int type1 = frame1.type();
                int type2 = frame2.type();

                // Передаём метаданные по сокету
                boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::write(socket, boost::asio::buffer(&type1, sizeof(type1)));
                boost::asio::write(socket, boost::asio::buffer(&type2, sizeof(type2)));

                boost::asio::write(socket, boost::asio::buffer(&cam1.currentFrame, sizeof(cam1.currentFrame)));
                boost::asio::write(socket, boost::asio::buffer(&cam2.currentFrame, sizeof(cam2.currentFrame)));

                // Размер сжатых данных первого кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.compressedSize, sizeof(cCoder_1.compressedSize)));

                // Размер сжатых данных второго кадра
                boost::asio::write(socket, boost::asio::buffer(&cCoder_2.compressedSize, sizeof(cCoder_2.compressedSize)));

                // Мы считаем, что исходные данные могут быть разные по размеру
                boost::asio::write(socket, boost::asio::buffer(&cCoder_1.originalSize, sizeof(cCoder_1.originalSize)));
                boost::asio::write(socket, boost::asio::buffer(&cCoder_2.originalSize, sizeof(cCoder_2.originalSize)));

                // Отправка данных клиенту кадр 1
                boost::asio::write(socket, boost::asio::buffer(cCoder_1.compressedData));
                // Отправка данных клиенту кадр 2
                boost::asio::write(socket, boost::asio::buffer(cCoder_2.compressedData));

                t4 = std::chrono::steady_clock::now();  // После передачи

                std::cout << " get image frame 1: " << cam1.timeToFrame << " get image frame 2: " << cam2.timeToFrame
                          << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                          << " uncompressed data: " << cCoder_1.originalSize + cCoder_2.originalSize
                          << " compressed data: " << cCoder_1.compressedSize + cCoder_2.compressedSize << " koef: "
                          << static_cast<double>(cCoder_1.originalSize + cCoder_2.originalSize) /
                                 static_cast<double>(cCoder_1.compressedSize + cCoder_2.compressedSize)
                          << std::endl;
            }
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error in work: " << ex.what() << std::endl;
        // Останавливаем потоки
        cam1.running = false;
        cam2.running = false;
        if (cam1Thread.joinable()) cam1Thread.join();
        if (cam2Thread.joinable()) cam2Thread.join();

        cam1.cap.release();
        cam2.cap.release();

        cv::destroyAllWindows();
    }
}

#pragma endregion
