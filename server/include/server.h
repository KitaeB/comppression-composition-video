#pragma once 

#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <mutex>
#include <thread>

#define VIDEO_WEIGHT 640
#define VIDEO_HEIGHT 480

using boost::asio::ip::tcp;

//Класс TCP сервера
class TcpServer 
{
public:
    TcpServer(const uint16_t port);
    void NewConnect();
    bool TestConnect();
    tcp::socket& GetSocket();

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
};

// Функция захвата кадров для отдельной камеры
void captureFrames(CameraState& camState, int camIndex);


#pragma region methods

void lz4_concat_noprime(tcp::socket &socket, CameraState &cam1, CameraState &cam2);

#pragma endregion