
#include "client.h"
#include "decomression.h"

#pragma region lz4

void lz4_concat_noprime(tcp::socket &socket) {
  int rows, cols, type;
  cv::Mat frame;
  std::vector<char> compressed_data, uncompressed_data;
  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  std::cout << "point 1 " << std::endl;
  try {
    while (true) {
      t0 = std::chrono::high_resolution_clock::now(); // До получения данных
      // Читаем метаданные
      boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

      int compressed_size;
      boost::asio::read(socket, boost::asio::buffer(&compressed_size,
                                                    sizeof(compressed_size)));

      int uncompressed_size;
      boost::asio::read(socket, boost::asio::buffer(&uncompressed_size,
                                                    sizeof(uncompressed_size)));

      // Читаем данные кадра
      compressed_data.resize(compressed_size);
      boost::asio::read(socket, boost::asio::buffer(compressed_data));

      t1 = std::chrono::high_resolution_clock::now(); // После получения данных

      frame = cv::Mat(rows, cols, type);
      if (lz4_decompress(compressed_data, uncompressed_data,
                         uncompressed_size)) {
        frame = convertFromCleanDataChar(uncompressed_data, frame.rows,
                                         frame.cols, frame.type());
      }
      t2 = std::chrono::high_resolution_clock::now(); // После сжатия
      cv::imshow("webcam", frame);

      t3 = std::chrono::high_resolution_clock::now(); // После отображения

      std::cout
          << " get data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " decompress data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0)
                     .count()
          << " uncompressed data: " << uncompressed_size
          << " compressed data: " << compressed_size << " koef: "
          << static_cast<double>(uncompressed_size) /
                 static_cast<double>(compressed_size)
          << std::endl;
      // Обязательно waitKey
      if (cv::waitKey(1) == 27) { // Нажал ESC
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
  uint16_t currentFrame;
  cv::Mat frame, prevFrame;
  std::vector<char> compressed_data, uncompressed_data;
  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  std::cout << "point 2 " << std::endl;
  try {
    while (true) {
      t0 = std::chrono::high_resolution_clock::now(); // До получения данных
      // Читаем метаданные
      boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
      boost::asio::read(
          socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

      int compressed_size;
      boost::asio::read(socket, boost::asio::buffer(&compressed_size,
                                                    sizeof(compressed_size)));

      int uncompressed_size;
      boost::asio::read(socket, boost::asio::buffer(&uncompressed_size,
                                                    sizeof(uncompressed_size)));

      // Читаем данные кадра
      compressed_data.resize(compressed_size);
      boost::asio::read(socket, boost::asio::buffer(compressed_data));

      t1 = std::chrono::high_resolution_clock::now(); // После получения данных

      frame = cv::Mat(rows, cols, type);
      if (lz4_decompress(compressed_data, uncompressed_data,
                         uncompressed_size)) {
        frame = convertFromCleanDataChar(uncompressed_data, frame.rows,
                                         frame.cols, frame.type());
        if ((currentFrame % 30) > 0 && !prevFrame.empty())
          frame = frameAddiiton(frame, prevFrame);
        prevFrame = frame.clone();
      }
      t2 = std::chrono::high_resolution_clock::now(); // После сжатия
      cv::imshow("webcam", frame);

      t3 = std::chrono::high_resolution_clock::now(); // После отображения

      std::cout
          << " get data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " decompress data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0)
                     .count()
          << " uncompressed data: " << uncompressed_size
          << " compressed data: " << compressed_size << " koef: "
          << static_cast<double>(uncompressed_size) /
                 static_cast<double>(compressed_size)
          << std::endl;
      // Обязательно waitKey
      if (cv::waitKey(1) == 27) { // Нажал ESC
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
  cv::Mat frame1, frame2;
  std::vector<char> compressed_data_frame_1, compressed_data_frame_2,
      uncompressed_data_frame_1, uncompressed_data_frame_2;
  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  std::cout << "point 3 " << std::endl;
  try {
    while (true) {
      t0 = std::chrono::high_resolution_clock::now(); // До получения данных
      // Читаем метаданные
      boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

      int compressed_size_frame_1;
      boost::asio::read(socket,
                        boost::asio::buffer(&compressed_size_frame_1,
                                            sizeof(compressed_size_frame_1)));

      int compressed_size_frame_2;
      boost::asio::read(socket,
                        boost::asio::buffer(&compressed_size_frame_2,
                                            sizeof(compressed_size_frame_2)));

      int uncompressed_size;
      boost::asio::read(socket, boost::asio::buffer(&uncompressed_size,
                                                    sizeof(uncompressed_size)));

      // Читаем данные первого кадра
      compressed_data_frame_1.resize(compressed_size_frame_1);
      boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_1));

      // Читаем данные второго кадра
      compressed_data_frame_2.resize(compressed_size_frame_2);
      boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_2));

      t1 = std::chrono::high_resolution_clock::now(); // После получения данных

      frame1 = cv::Mat(rows, cols, type);
      if (lz4_decompress(compressed_data_frame_1, uncompressed_data_frame_1,
                         uncompressed_size)) {
        frame1 = convertFromCleanDataChar(
            uncompressed_data_frame_1, frame1.rows, frame1.cols, frame1.type());
        cv::imshow("webcam1", frame1);
      }

      frame2 = cv::Mat(rows, cols, type);
      if (lz4_decompress(compressed_data_frame_2, uncompressed_data_frame_2,
                         uncompressed_size)) {
        frame2 = convertFromCleanDataChar(
            uncompressed_data_frame_2, frame2.rows, frame2.cols, frame2.type());
        cv::imshow("webcam2", frame2);
      }
      t2 = std::chrono::high_resolution_clock::now(); // После сжатия

      t3 = std::chrono::high_resolution_clock::now(); // После отображения

      std::cout
          << " get data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " decompress data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0)
                     .count()
          << " uncompressed data: " << uncompressed_size * 2
          << " compressed data: "
          << compressed_size_frame_1 + compressed_size_frame_2 << " koef: "
          << static_cast<double>(uncompressed_size * 2) /
                 static_cast<double>(compressed_size_frame_1 +
                                     compressed_size_frame_2)
          << std::endl;
      // Обязательно waitKey
      if (cv::waitKey(1) == 27) { // Нажал ESC
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
  cv::Mat frame1, frame2, prevFrame1, prevFrame2;
  std::vector<char> compressed_data_frame_1, compressed_data_frame_2,
      uncompressed_data_frame_1, uncompressed_data_frame_2;
  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  std::cout << "point 4 " << std::endl;
  try {
    while (true) {
      t0 = std::chrono::high_resolution_clock::now(); // До получения данных
      // Читаем метаданные
      boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::read(socket, boost::asio::buffer(&type1, sizeof(type1)));
      boost::asio::read(socket, boost::asio::buffer(&type2, sizeof(type2)));

      int compressed_size_frame_1;
      boost::asio::read(socket,
                        boost::asio::buffer(&compressed_size_frame_1,
                                            sizeof(compressed_size_frame_1)));

      int compressed_size_frame_2;
      boost::asio::read(socket,boost::asio::buffer(&compressed_size_frame_2,sizeof(compressed_size_frame_2)));

      int uncompressed_size_1;
      boost::asio::read(socket, boost::asio::buffer(&uncompressed_size_1,sizeof(uncompressed_size_1)));
                                                    
      int uncompressed_size_2;
      boost::asio::read(socket, boost::asio::buffer(&uncompressed_size_2,sizeof(uncompressed_size_2)));

      int currentFrame1;
      boost::asio::read(socket, boost::asio::buffer(&currentFrame1,sizeof(currentFrame1)));
                                                    
      int currentFrame2;
      boost::asio::read(socket, boost::asio::buffer(&currentFrame2,sizeof(currentFrame2)));

      // Читаем данные первого кадра
      compressed_data_frame_1.resize(compressed_size_frame_1);
      boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_1));

      // Читаем данные второго кадра
      compressed_data_frame_2.resize(compressed_size_frame_2);
      boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_2));

      t1 = std::chrono::high_resolution_clock::now(); // После получения данных

      frame1 = cv::Mat(rows, cols, type1);
      if (lz4_decompress(compressed_data_frame_1, uncompressed_data_frame_1,
                         uncompressed_size_1)) {
        frame1 = convertFromCleanDataChar(
            uncompressed_data_frame_1, frame1.rows, frame1.cols, frame1.type());
            
        if ((currentFrame1 % 30) > 0 && !prevFrame1.empty())
          frame1 = frameAddiiton(frame1, prevFrame1);
        prevFrame1 = frame1.clone();
        cv::imshow("webcam1", frame1);
      }

      frame2 = cv::Mat(rows, cols, type2);
      if (lz4_decompress(compressed_data_frame_2, uncompressed_data_frame_2,
                         uncompressed_size_2)) {
        frame2 = convertFromCleanDataChar(
            uncompressed_data_frame_2, frame2.rows, frame2.cols, frame2.type());
            
        if ((currentFrame2 % 30) > 0 && !prevFrame2.empty())
          frame2 = frameAddiiton(frame2, prevFrame2);
        prevFrame2 = frame2.clone();
        cv::imshow("webcam2", frame2);
      }
      t2 = std::chrono::high_resolution_clock::now(); // После сжатия

      t3 = std::chrono::high_resolution_clock::now(); // После отображения

      std::cout
          << " get data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " decompress data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0)
                     .count()
          << " uncompressed data: " << uncompressed_size_1 + uncompressed_size_2
          << " compressed data: "
          << compressed_size_frame_1 + compressed_size_frame_2 << " koef: "
          << static_cast<double>(uncompressed_size_2 * 2) /
                 static_cast<double>(compressed_size_frame_1 +
                                     compressed_size_frame_2)
          << std::endl;
      // Обязательно waitKey
      if (cv::waitKey(1) == 27) { // Нажал ESC
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
  cv::Mat frame;
  std::vector<Bytef> compressed_data, uncompressed_data;
  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  std::cout << "point 5 " << std::endl;
  try {
    while (true) {
      t0 = std::chrono::high_resolution_clock::now(); // До получения данных
      // Читаем метаданные
      boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

      int compressed_size;
      boost::asio::read(socket, boost::asio::buffer(&compressed_size,
                                                    sizeof(compressed_size)));

      int uncompressed_size;
      boost::asio::read(socket, boost::asio::buffer(&uncompressed_size,
                                                    sizeof(uncompressed_size)));

      // Читаем данные кадра
      compressed_data.resize(compressed_size);
      boost::asio::read(socket, boost::asio::buffer(compressed_data));

      t1 = std::chrono::high_resolution_clock::now(); // После получения данных

      frame = cv::Mat(rows, cols, type);
      if (zlib_decompress(compressed_data, uncompressed_data,
                          uncompressed_size) == 0) {
        frame = convertFromCleanDataBytef(uncompressed_data, frame.rows,
                                          frame.cols, frame.type());
      }
      t2 = std::chrono::high_resolution_clock::now(); // После сжатия
      cv::imshow("webcam", frame);

      t3 = std::chrono::high_resolution_clock::now(); // После отображения

      std::cout
          << " get data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " decompress data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0)
                     .count()
          << " uncompressed data: " << uncompressed_size
          << " compressed data: " << compressed_size << " koef: "
          << static_cast<double>(uncompressed_size) /
                 static_cast<double>(compressed_size)
          << std::endl;
      // Обязательно waitKey
      if (cv::waitKey(1) == 27) { // Нажал ESC
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
  std::vector<Bytef> compressed_data_frame_1, compressed_data_frame_2,
      uncompressed_data_frame_1, uncompressed_data_frame_2;
  // Объявим временные метки
  std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
  std::cout << "point 7 " << std::endl;
  try {
    while (true) {
      t0 = std::chrono::high_resolution_clock::now(); // До получения данных
      // Читаем метаданные
      boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

      int compressed_size_frame_1;
      boost::asio::read(socket,
                        boost::asio::buffer(&compressed_size_frame_1,
                                            sizeof(compressed_size_frame_1)));

      int compressed_size_frame_2;
      boost::asio::read(socket,
                        boost::asio::buffer(&compressed_size_frame_2,
                                            sizeof(compressed_size_frame_2)));

      int uncompressed_size;
      boost::asio::read(socket, boost::asio::buffer(&uncompressed_size,
                                                    sizeof(uncompressed_size)));

      // Читаем данные первого кадра
      compressed_data_frame_1.resize(compressed_size_frame_1);
      boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_1));

      // Читаем данные второго кадра
      compressed_data_frame_2.resize(compressed_size_frame_2);
      boost::asio::read(socket, boost::asio::buffer(compressed_data_frame_2));

      t1 = std::chrono::high_resolution_clock::now(); // После получения данных

      frame1 = cv::Mat(rows, cols, type);
      if (zlib_decompress(compressed_data_frame_1, uncompressed_data_frame_1,
                          uncompressed_size) == 0) {
        frame1 = convertFromCleanDataBytef(
            uncompressed_data_frame_1, frame1.rows, frame1.cols, frame1.type());
        cv::imshow("webcam1", frame1);
      }

      frame2 = cv::Mat(rows, cols, type);
      if (zlib_decompress(compressed_data_frame_2, uncompressed_data_frame_2,
                          uncompressed_size) == 0) {
        frame2 = convertFromCleanDataBytef(
            uncompressed_data_frame_2, frame2.rows, frame2.cols, frame2.type());
        cv::imshow("webcam2", frame2);
      }
      t2 = std::chrono::high_resolution_clock::now(); // После сжатия

      t3 = std::chrono::high_resolution_clock::now(); // После отображения

      std::cout
          << " get data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                 .count()
          << " decompress data: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                 .count()
          << " get image: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                 .count()
          << " FPS: "
          << 1000 /
                 std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0)
                     .count()
          << " uncompressed data: " << uncompressed_size * 2
          << " compressed data: "
          << compressed_size_frame_1 + compressed_size_frame_2 << " koef: "
          << static_cast<double>(uncompressed_size * 2) /
                 static_cast<double>(compressed_size_frame_1 +
                                     compressed_size_frame_2)
          << std::endl;
      // Обязательно waitKey
      if (cv::waitKey(1) == 27) { // Нажал ESC
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
      t0 = std::chrono::high_resolution_clock::now(); // До получения данных
      // Читаем метаданные
      boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
      boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
      boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

      int compressed_size;
      boost::asio::read(socket, boost::asio::buffer(&compressed_size, sizeof(compressed_size)));

      // Читаем данные кадра
      compressed_data.resize(compressed_size);
      boost::asio::read(socket, boost::asio::buffer(compressed_data));

      t1 = std::chrono::high_resolution_clock::now(); // После получения данных

      frame = cv::Mat(rows, cols, type);
      if (!aom_decompress(compressed_data, frame)) {
        std::cout << "decompress error!!";
        continue;
      }
      t2 = std::chrono::high_resolution_clock::now(); // После сжатия
      cv::imshow("webcam", frame);

      t3 = std::chrono::high_resolution_clock::now(); // После отображения

      std::cout
          << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
          << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
          << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
          << " compressed data: " << compressed_size << std::endl;
      // Обязательно waitKey
      if (cv::waitKey(1) == 27) { // Нажал ESC
        break;
      }
    }
  } catch (...) {
    std::cerr << "Error in receiving frame. Closing..." << std::endl;
    cv::destroyAllWindows();
  }
}

#pragma endregion