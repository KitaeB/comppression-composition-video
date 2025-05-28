#include "server.h"
#include "compression.h"
#include <aom/aom_codec.h>
#include <exception>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <ostream>

#pragma region tcp_server

TcpServer::TcpServer(const uint16_t port)
    : socket(io_context), acceptor(io_context, tcp::endpoint(tcp::v4(), port)),
      PORT(port) {}

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
void captureFrames(CameraState& camState, int camIndex) {
    camState.cap.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_WEIGHT);
    camState.cap.set(cv::CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);

    if (!camState.cap.isOpened()) {
        std::cerr << "Failed to open camera " << camIndex << std::endl;
        camState.running = false;
        return;
    }

    cv::Mat frame;
    while (camState.running) {
        if (camState.cap.read(frame) && !frame.empty()) {
            std::lock_guard<std::mutex> lock(camState.frameMutex);
            camState.lastFrame = frame.clone();
            camState.frameReady = true;
        } else {
            std::cerr << "Failed to capture frame from camera " << camIndex << std::endl;
        }
        // Небольшая задержка, чтобы не перегружать CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

#pragma endregion

#pragma region lz4

void lz4_concat_noprime(tcp::socket &socket, CameraState &cam1,
                        CameraState &cam2) {
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
  // основной цикл
  while (true) {
    t0 = std::chrono::high_resolution_clock::now(); // До получения картинки
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
        frame1 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
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
        frame2 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }

    if (!hasNewFrame) {
      // Если нет новых кадров, ждем немного
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    // После получения
    t1 = std::chrono::high_resolution_clock::now();

    // производим соединение кадров
    cv::hconcat(frame1, frame2, frame);

    auto last_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

    if (last_duration > 20 && lz4Coder.acceleration < 100)
      lz4Coder.acceleration++;
    else if (last_duration < 15 && lz4Coder.acceleration > 1)
      lz4Coder.acceleration--;
    t2 = std::chrono::high_resolution_clock::now(); // Изменение размера и
                                                    // объединение

    // Сожмём данные с lz4-fast
    if (lz4Coder.lz4_compress_fast(frame) > 0) {
      t3 = std::chrono::high_resolution_clock::now(); // После сжатия
      // Объявим метаданные передаваеммого кадра
      int rows = frame.rows;
      int cols = frame.cols;
      int type = frame.type();

      // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " << type
      // << std::endl;

      // Передаём метаданные по сокету
      boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

      boost::asio::write(socket,
                         boost::asio::buffer(&lz4Coder.compressedSize,
                                             sizeof(lz4Coder.compressedSize)));

      boost::asio::write(
          socket, boost::asio::buffer(&lz4Coder.uncompressedSize,
                                      sizeof(lz4Coder.uncompressedSize)));

      // Отправка данных клиенту
      boost::asio::write(socket, boost::asio::buffer(lz4Coder.compressedData));

      t4 = std::chrono::high_resolution_clock::now(); // После передачи

      std::cout
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " convert image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " compress: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " send: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0)
                     .count()
          << " acceleration: " << lz4Coder.acceleration
          << " uncompressed data: " << lz4Coder.uncompressedSize
          << " compressed data: " << lz4Coder.compressedSize << " koef: "
          << static_cast<double>(lz4Coder.uncompressedSize) /
                 static_cast<double>(lz4Coder.compressedSize)
          << std::endl;
      if (cv::waitKey(1) == 27) { // Esc key to stop
        break;
      }
    }
  }
  // Останавливаем потоки
  cam1.running = false;
  cam2.running = false;
  if (cam1Thread.joinable())
    cam1Thread.join();
  if (cam2Thread.joinable())
    cam2Thread.join();

  cam1.cap.release();
  cam2.cap.release();

  cv::destroyAllWindows();
}

void lz4_concat_prime(tcp::socket &socket, CameraState &cam1,
                      CameraState &cam2) {
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
  // основной цикл
  while (true) {
    t0 = std::chrono::high_resolution_clock::now(); // До получения картинки
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
        frame1 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
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
        frame2 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }
    if (!hasNewFrame) {
      // Если нет новых кадров, ждем немного
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // После получения
    t1 = std::chrono::high_resolution_clock::now();

    // производим соединение кадров
    cv::hconcat(frame1, frame2, frame);

    // cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);
    tempFrame = frame.clone();

    if ((currentFrame % 30) > 0 && !prevFrame.empty()) {
      frame = frameSubstraction(frame, prevFrame); // Производим вычитания
    }

    prevFrame = tempFrame.clone();
    auto last_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

    if (last_duration > 20 && lz4Coder.acceleration < 200)
      lz4Coder.acceleration++;
    else if (last_duration < 15 && lz4Coder.acceleration > 1)
      lz4Coder.acceleration--;

    t2 = std::chrono::high_resolution_clock::now(); // Изменение размера и
                                                    // объединение

    // Сожмём данные с zlib-default
    if (lz4Coder.lz4_compress_fast(frame) > 0) {
      t3 = std::chrono::high_resolution_clock::now(); // После сжатия
      // Объявим метаданные передаваеммого кадра
      int rows = frame.rows;
      int cols = frame.cols;
      int type = frame.type();

      // Передаём метаданные по сокету
      boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));
      boost::asio::write(
          socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

      boost::asio::write(socket,
                         boost::asio::buffer(&lz4Coder.compressedSize,
                                             sizeof(lz4Coder.compressedSize)));

      boost::asio::write(
          socket, boost::asio::buffer(&lz4Coder.uncompressedSize,
                                      sizeof(lz4Coder.uncompressedSize)));

      // Отправка данных клиенту
      boost::asio::write(socket, boost::asio::buffer(lz4Coder.compressedData));

      t4 = std::chrono::high_resolution_clock::now(); // После передачи

      std::cout
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " convert image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " compress: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " send: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0)
                     .count()
          << " acceleration: " << lz4Coder.acceleration
          << " currentFrame: " << currentFrame
          << " uncompressed data: " << lz4Coder.uncompressedSize
          << " compressed data: " << lz4Coder.compressedSize << " koef: "
          << static_cast<double>(lz4Coder.uncompressedSize) /
                 static_cast<double>(lz4Coder.compressedSize)
          << std::endl;
      currentFrame++;

      if (cv::waitKey(1) == 27) { // Esc key to stop
        break;
      }
    }
  }
  // Останавливаем потоки
  cam1.running = false;
  cam2.running = false;
  if (cam1Thread.joinable())
    cam1Thread.join();
  if (cam2Thread.joinable())
    cam2Thread.join();

  cam1.cap.release();
  cam2.cap.release();

  cv::destroyAllWindows();
}

void lz4_noconcat_noprime(tcp::socket &socket, CameraState &cam1,
                          CameraState &cam2) {
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
  // основной цикл
  while (true) {
    t0 = std::chrono::high_resolution_clock::now(); // До получения картинки
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
        frame1 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
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
        frame2 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }

    if (!hasNewFrame) {
      // Если нет новых кадров, ждем немного
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    auto last_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

    if (last_duration > 20 && lz4Coder1.acceleration < 100)
      lz4Coder1.acceleration++;
    else if (last_duration < 15 && lz4Coder1.acceleration > 1)
      lz4Coder1.acceleration--;
    lz4Coder2.acceleration = lz4Coder1.acceleration;

    t2 = std::chrono::high_resolution_clock::now(); // Изменение размера и
                                                    // объединение

    // Сожмём данные с zlib-default
    if ((lz4Coder1.lz4_compress_fast(frame1) > 0) &&
        (lz4Coder2.lz4_compress_fast(frame2) > 0)) {
      t3 = std::chrono::high_resolution_clock::now(); // После сжатия
      // Объявим метаданные передаваеммого кадра
      int rows = frame1.rows;
      int cols = frame1.cols;
      int type = frame1.type();

      // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " << type
      // << std::endl;

      // Передаём метаданные по сокету
      boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

      // Размер сжатых данных первого кадра
      boost::asio::write(socket,
                         boost::asio::buffer(&lz4Coder1.compressedSize,
                                             sizeof(lz4Coder1.compressedSize)));

      // Размер сжатых данных второго кадра
      boost::asio::write(socket,
                         boost::asio::buffer(&lz4Coder2.compressedSize,
                                             sizeof(lz4Coder2.compressedSize)));

      // Мы считаем, что исходные данные одинаковые по размеры
      boost::asio::write(
          socket, boost::asio::buffer(&lz4Coder1.uncompressedSize,
                                      sizeof(lz4Coder1.uncompressedSize)));

      // Отправка данных клиенту кадр 1
      boost::asio::write(socket, boost::asio::buffer(lz4Coder1.compressedData));
      // Отправка данных клиенту кадр 2
      boost::asio::write(socket, boost::asio::buffer(lz4Coder2.compressedData));

      t4 = std::chrono::high_resolution_clock::now(); // После передачи

      std::cout
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0)
                 .count()
          << " compress: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " send: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0)
                     .count()
          << " acceleration: " << lz4Coder1.acceleration
          << " uncompressed data: " << lz4Coder1.uncompressedSize * 2
          << " compressed data: "
          << lz4Coder1.compressedSize + lz4Coder2.compressedSize << " koef: "
          << static_cast<double>(lz4Coder1.uncompressedSize * 2) /
                 static_cast<double>(lz4Coder1.compressedSize +
                                     lz4Coder2.compressedSize)
          << std::endl;
      if (cv::waitKey(1) == 27) { // Esc key to stop
        break;
      }
    }
  }
  // Останавливаем потоки
  cam1.running = false;
  cam2.running = false;
  if (cam1Thread.joinable())
    cam1Thread.join();
  if (cam2Thread.joinable())
    cam2Thread.join();

  cam1.cap.release();
  cam2.cap.release();

  cv::destroyAllWindows();
}

void lz4_noconcat_prime(tcp::socket &socket, CameraState &cam1,
                        CameraState &cam2) {
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
  // основной цикл
  while (true) {
    t0 = std::chrono::high_resolution_clock::now(); // До получения картинки
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
        frame1 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
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
        frame2 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }

    if (!hasNewFrame) {
      // Если нет новых кадров, ждем немного
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    auto last_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

    if (last_duration > 20 && lz4Coder1.acceleration < 100)
      lz4Coder1.acceleration++;
    else if (last_duration < 15 && lz4Coder1.acceleration > 1)
      lz4Coder1.acceleration--;
    lz4Coder2.acceleration = lz4Coder1.acceleration;

    t2 = std::chrono::high_resolution_clock::now(); // Изменение размера и
                                                    // объединение

    tempFrame1 = frame1.clone();
    // cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);
    if ((cam1.currentFrame % 30) > 0 && !prevFrame1.empty()) {
      frame1 = frameSubstraction(tempFrame1, prevFrame1); // Производим
                                                          // вычитания
    }
    prevFrame1 = tempFrame1.clone();

    tempFrame2 = frame2.clone();
    if ((cam2.currentFrame % 30) > 0 && !prevFrame2.empty()) {
      frame2 = frameSubstraction(tempFrame2, prevFrame2); // Производим
                                                          // вычитания
    }
    prevFrame2 = tempFrame2.clone();

    // Сожмём данные с zlib-default
    if ((lz4Coder1.lz4_compress_fast(frame1) > 0) &&
        (lz4Coder2.lz4_compress_fast(frame2) > 0)) {
      t3 = std::chrono::high_resolution_clock::now(); // После сжатия
      // Объявим метаданные передаваеммого кадра
      int rows = frame1.rows;
      int cols = frame1.cols;
      int type1 = frame1.type();
      int type2 = frame2.type();

      // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " << type
      // << std::endl;

      // Передаём метаданные по сокету
      boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::write(socket, boost::asio::buffer(&type1, sizeof(type1)));
      boost::asio::write(socket, boost::asio::buffer(&type2, sizeof(type2)));

      // Размер сжатых данных первого кадра
      boost::asio::write(socket,
                         boost::asio::buffer(&lz4Coder1.compressedSize,
                                             sizeof(lz4Coder1.compressedSize)));

      // Размер сжатых данных второго кадра
      boost::asio::write(socket,
                         boost::asio::buffer(&lz4Coder2.compressedSize,
                                             sizeof(lz4Coder2.compressedSize)));

      // Мы не можем считать, что исходные данные одинаковые по размеру,
      // поскольку у нас один кадр может быть ключевым, а другой нет
      boost::asio::write(
          socket, boost::asio::buffer(&lz4Coder1.uncompressedSize,
                                      sizeof(lz4Coder1.uncompressedSize)));

      boost::asio::write(
          socket, boost::asio::buffer(&lz4Coder2.uncompressedSize,
                                      sizeof(lz4Coder2.uncompressedSize)));

      // Также надо отправить данные о том у нас кадр ключеввой или нет
      boost::asio::write(
          socket,
          boost::asio::buffer(&cam1.currentFrame, sizeof(cam1.currentFrame)));
      boost::asio::write(
          socket,
          boost::asio::buffer(&cam2.currentFrame, sizeof(cam2.currentFrame)));

      // Отправка данных клиенту кадр 1
      boost::asio::write(socket, boost::asio::buffer(lz4Coder1.compressedData));
      // Отправка данных клиенту кадр 2
      boost::asio::write(socket, boost::asio::buffer(lz4Coder2.compressedData));

      t4 = std::chrono::high_resolution_clock::now(); // После передачи

      std::cout
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0)
                 .count()
          << " compress: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " send: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0)
                     .count()
          << " acceleration: " << lz4Coder1.acceleration
          << " uncompressed data: "
          << lz4Coder1.uncompressedSize + lz4Coder2.uncompressedSize
          << " compressed data: "
          << lz4Coder2.compressedSize + lz4Coder1.compressedSize << " koef: "
          << static_cast<double>(lz4Coder1.uncompressedSize +
                                 lz4Coder2.uncompressedSize) /
                 static_cast<double>(lz4Coder2.compressedSize +
                                     lz4Coder1.compressedSize)
          << std::endl;

      if (cv::waitKey(1) == 27) { // Esc key to stop
        break;
      }
    }
  }
  // Останавливаем потоки
  cam1.running = false;
  cam2.running = false;
  if (cam1Thread.joinable())
    cam1Thread.join();
  if (cam2Thread.joinable())
    cam2Thread.join();

  cam1.cap.release();
  cam2.cap.release();

  cv::destroyAllWindows();
}

#pragma endregion

#pragma region zlib

void zlib_concat_noprime(tcp::socket &socket, CameraState &cam1,
                         CameraState &cam2) {
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
  std::vector<Bytef> compressed_data, uncompressed_data;

  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  // основной цикл
  while (true) {
    t0 = std::chrono::high_resolution_clock::now(); // До получения картинки
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
        frame1 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
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
        frame2 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }

    if (!hasNewFrame) {
      // Если нет новых кадров, ждем немного
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // После получения
    t1 = std::chrono::high_resolution_clock::now();

    // производим соединение кадров
    cv::hconcat(frame1, frame2, frame);

    auto last_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

    t2 = std::chrono::high_resolution_clock::now(); // Изменение размера и
                                                    // объединение

    // Преобразуем данные в vector<Bytef>
    convertToCleanDataBytef(frame, uncompressed_data);
    // Сожмём данные с zlib-default
    if (zlib_compress_fast(uncompressed_data, compressed_data) > 0) {
      t3 = std::chrono::high_resolution_clock::now(); // После сжатия
      // Объявим метаданные передаваеммого кадра
      int rows = frame.rows;
      int cols = frame.cols;
      int type = frame.type();

      // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " << type
      // << std::endl;

      // Передаём метаданные по сокету
      boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

      int compressed_size = compressed_data.size();
      boost::asio::write(socket, boost::asio::buffer(&compressed_size,
                                                     sizeof(compressed_size)));

      int uncompressed_size = uncompressed_data.size();
      boost::asio::write(
          socket,
          boost::asio::buffer(&uncompressed_size, sizeof(uncompressed_size)));

      // Отправка данных клиенту
      boost::asio::write(socket, boost::asio::buffer(compressed_data));

      t4 = std::chrono::high_resolution_clock::now(); // После передачи
      prevFrame = frame.clone();

      std::cout
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " convert image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " compress: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " send: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0)
                     .count()
          << " uncompressed data: " << uncompressed_size
          << " compressed data: " << compressed_size << " koef: "
          << static_cast<double>(uncompressed_size) /
                 static_cast<double>(compressed_size)
          << std::endl;
      if (cv::waitKey(1) == 27) { // Esc key to stop
        break;
      }
    }
  }
  // Останавливаем потоки
  cam1.running = false;
  cam2.running = false;
  if (cam1Thread.joinable())
    cam1Thread.join();
  if (cam2Thread.joinable())
    cam2Thread.join();

  cam1.cap.release();
  cam2.cap.release();

  cv::destroyAllWindows();
}

void zlib_noconcat_noprime(tcp::socket &socket, CameraState &cam1,
                           CameraState &cam2) {
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
  std::vector<Bytef> compressed_data_frame_1, compressed_data_frame_2,
      uncompressed_data_frame_1, uncompressed_data_frame_2;
  uint16_t acceleration = 1;

  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  // основной цикл
  while (true) {
    t0 = std::chrono::high_resolution_clock::now(); // До получения картинки
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
        frame1 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
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
        frame2 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }

    if (!hasNewFrame) {
      // Если нет новых кадров, ждем немного
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    t2 = std::chrono::high_resolution_clock::now(); // Изменение размера и
                                                    // объединение

    // Преобразуем данные в vector<Bytef>
    convertToCleanDataBytef(frame1, uncompressed_data_frame_1);

    // Преобразуем данные в vector<Bytef>
    convertToCleanDataBytef(frame2, uncompressed_data_frame_2);

    // Сожмём данные с zlib-fast
    if ((zlib_compress_fast(uncompressed_data_frame_1,
                            compressed_data_frame_1) > 0) &&
        (zlib_compress_fast(uncompressed_data_frame_2,
                            compressed_data_frame_2) > 0)) {
      t3 = std::chrono::high_resolution_clock::now(); // После сжатия
      // Объявим метаданные передаваеммого кадра
      int rows = frame1.rows;
      int cols = frame1.cols;
      int type = frame1.type();

      // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " << type
      // << std::endl;

      // Передаём метаданные по сокету
      boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

      // Размер сжатых данных первого кадра
      int compressed_size_frame_1 = compressed_data_frame_1.size();
      boost::asio::write(socket,
                         boost::asio::buffer(&compressed_size_frame_1,
                                             sizeof(compressed_size_frame_1)));

      // Размер сжатых данных второго кадра
      int compressed_size_frame_2 = compressed_data_frame_2.size();
      boost::asio::write(socket,
                         boost::asio::buffer(&compressed_size_frame_2,
                                             sizeof(compressed_size_frame_2)));

      // Мы считаем, что исходные данные одинаковые по размеры
      int uncompressed_size = uncompressed_data_frame_1.size();
      boost::asio::write(
          socket,
          boost::asio::buffer(&uncompressed_size, sizeof(uncompressed_size)));

      // Отправка данных клиенту кадр 1
      boost::asio::write(socket, boost::asio::buffer(compressed_data_frame_1));
      // Отправка данных клиенту кадр 2
      boost::asio::write(socket, boost::asio::buffer(compressed_data_frame_2));

      t4 = std::chrono::high_resolution_clock::now(); // После передачи

      std::cout
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0)
                 .count()
          << " compress: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " send: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0)
                     .count()
          << " acceleration: " << acceleration
          << " uncompressed data: " << uncompressed_size * 2
          << " compressed data: "
          << compressed_size_frame_1 + compressed_size_frame_2 << " koef: "
          << static_cast<double>(uncompressed_size * 2) /
                 static_cast<double>(compressed_size_frame_1 +
                                     compressed_size_frame_2)
          << std::endl;
      if (cv::waitKey(1) == 27) { // Esc key to stop
        break;
      }
    }
  }
  // Останавливаем потоки
  cam1.running = false;
  cam2.running = false;
  if (cam1Thread.joinable())
    cam1Thread.join();
  if (cam2Thread.joinable())
    cam2Thread.join();

  cam1.cap.release();
  cam2.cap.release();

  cv::destroyAllWindows();
}

#pragma endregion

#pragma region aom

void aom_concat_noprime(tcp::socket &socket, CameraState &cam1,
                        CameraState &cam2) {
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
  std::vector<uint8_t> compressed_data;
  static int frame_count = 0;

  // Енкодер
  aom_codec_ctx_t encoder = {};
  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  // основной цикл
  while (true) {
    t0 = std::chrono::high_resolution_clock::now(); // До получения картинки
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
        frame1 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
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
        frame2 =
            cv::Mat(VIDEO_HEIGHT, VIDEO_WEIGHT, CV_8UC3, cv::Scalar(0, 0, 0));
      }
    }

    if (!hasNewFrame) {
      // Если нет новых кадров, ждем немного
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // После получения
    t1 = std::chrono::high_resolution_clock::now();

    // производим соединение кадров
    cv::hconcat(frame1, frame2, frame);

    if (!encoder.name) {
      std::cout << "Initialize encoder..." << std::endl;
      encoder = create_aom_encoder(frame, false, 30, 0.1);
    }

    t2 = std::chrono::high_resolution_clock::now(); // Изменение размера и
                                                    // объединение
    try {
      // Сожмём данные с zlib-default
      if (aom_compress_loseless(frame, compressed_data, encoder,
                                frame_count++) > 0) {
        t3 = std::chrono::high_resolution_clock::now(); // После сжатия
        // Объявим метаданные передаваеммого кадра
        int rows = frame.rows;
        int cols = frame.cols;
        int type = frame.type();

        // std::cout << "rows: " << rows << " Cols: " << cols << " Type: " <<
        // type << std::endl;

        // Передаём метаданные по сокету
        boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
        boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
        boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

        int compressed_size = compressed_data.size();
        boost::asio::write(
            socket,
            boost::asio::buffer(&compressed_size, sizeof(compressed_size)));

        // Отправка данных клиенту
        boost::asio::write(socket, boost::asio::buffer(compressed_data));

        t4 = std::chrono::high_resolution_clock::now(); // После передачи
        prevFrame = frame.clone();

        std::cout
            << " get image: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                   .count()
            << " convert image: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                   .count()
            << " compress: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                   .count()
            << " send: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                   .count()
            << " FPS: "
            << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 -
                                                                            t0)
                          .count()
            << " uncompressed data: " << frame.total() * frame.elemSize()
            << " compressed data: " << compressed_size << " koef: "
            << static_cast<double>(frame.total() * frame.elemSize()) /
                   static_cast<double>(compressed_size)
            << std::endl;
        if (cv::waitKey(1) == 27) { // Esc key to stop
          break;
        }
      } else {
        std::cout << "compress error";
      }
    } catch (const std::exception ex) {
      std::cerr << "Exception: " << ex.what() << std::endl;
    }
  }
  // Останавливаем потоки
  cam1.running = false;
  cam2.running = false;
  if (cam1Thread.joinable())
    cam1Thread.join();
  if (cam2Thread.joinable())
    cam2Thread.join();

  cam1.cap.release();
  cam2.cap.release();

  if (encoder.name) {
    aom_codec_destroy(&encoder);
  }

  cv::destroyAllWindows();
}

#pragma endregion