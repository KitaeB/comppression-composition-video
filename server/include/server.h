#pragma once

#include <opencv2/core/hal/interface.h>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>
#include <opencv2/videoio.hpp>

using boost::asio::ip::tcp;

#pragma region tcp_server

// Класс TCP сервера
class TcpServer {
public:
    TcpServer(const uint16_t port);
    void NewConnect();
    bool TestConnect();
    tcp::socket &GetSocket();

private:
    uint16_t PORT;
    boost::asio::io_context io_context;
    tcp::socket socket;
    tcp::acceptor acceptor;
};

// Структура для хранения состояния камеры
struct CameraState {
    cv::VideoCapture cap;
    cv::Mat lastFrame;
    std::atomic<bool> frameReady{false};
    std::mutex frameMutex;
    std::atomic<bool> running{true};

    int height = 720;
    int width = 1280;

    std::string file;
    int currentFrame = 0;

    // Конструкторы
    CameraState() = default;

    CameraState(const std::string &filename)
        : file(filename), cap(filename) {}

    CameraState(uint index, uint apiPreference) {
        cap.open(index, apiPreference);
    }

    // Запрещаем копирование
    CameraState(const CameraState &) = delete;
    CameraState &operator=(const CameraState &) = delete;

    // Разрешаем перемещение
    CameraState(CameraState &&other) noexcept
        : cap(std::move(other.cap)),
          lastFrame(std::move(other.lastFrame)),
          frameReady(other.frameReady.load()),
          running(other.running.load()),
          height(other.height),
          width(other.width),
          file(std::move(other.file)),
          currentFrame(other.currentFrame) {}

    CameraState &operator=(CameraState &&other) noexcept {
        if (this != &other) {
            cap = std::move(other.cap);
            lastFrame = std::move(other.lastFrame);
            frameReady.store(other.frameReady.load());
            running.store(other.running.load());
            height = other.height;
            width = other.width;
            file = std::move(other.file);
            currentFrame = other.currentFrame;
        }
        return *this;
    }
};


#pragma endregion

#pragma region common

// Функция захвата кадров для отдельной камеры
void captureFrames(CameraState &camState, int camIndex);

#pragma endregion

#pragma region methods

/* ============================================================================================================ */

void lz4_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void lz4_concat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void lz4_noconcat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void lz4_noconcat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

/* ============================================================================================================ */

void zlib_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void zlib_concat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void zlib_noconcat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void zlib_noconcat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

/* ============================================================================================================ */

void zstd_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void zstd_concat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void zstd_noconcat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

void zstd_noconcat_prime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

#pragma endregion
