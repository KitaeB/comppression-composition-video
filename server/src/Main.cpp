#include <boost/asio.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "server.h"

using boost::asio::ip::tcp;

int main(int argc, char *argv[]) {
  // Создаём сервер
  TcpServer server(9090);

  // Дополнительные параметры
  bool useVideo = false;
  int width = 1280;
  int height = 720;

  CameraState cam1;
  CameraState cam2;
  if (argc > 1) {
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "-uv") {
        useVideo = true;

      } else if (std::string(argv[i]) == "-fhd") {
        width = 1920;
        height = 1080;

      } else {
        std::cout << "Uncertain variable." << std::endl;
      }
    }
  }
  while (true) {
    if (useVideo) {
      // Подключаем видео
      cam1 = CameraState{"video/file_1.mp4"};
      cam2 = CameraState{"video/file_2.mp4"};
    } else {
      // Подключаем камеры
      // Кроссплатформенная инициализация камер
      #ifdef _WIN32
          // Windows — можно использовать DirectShow
          cam1 = CameraState{0, cv::CAP_DSHOW};
          cam2 = CameraState{1, cv::CAP_DSHOW};
      #else
          // Linux — предпочтительно V4L2
          cam1 = CameraState{0, cv::CAP_V4L2};
          cam2 = CameraState{1, cv::CAP_V4L2};
      #endif
    }

    int choice = 0;
    // Цикл для реалиализации переподключения для сервера, в случае ошибки
    try {
      // Инициализируем новое подключение
      server.NewConnect();

      // Получаем сокет нашего соединения
      tcp::socket &socket(server.GetSocket());

      // Выводим меню
      std::cout << "\n===  ===" << std::endl;
      std::cout << "1. lz4 compress, with concat, without prime frame "
                << std::endl;
      std::cout << "2. lz4 compress, with concat, with prime frame"
                << std::endl;
      std::cout << "3. lz4 compress, without concat, without prime frame"
                << std::endl;
      std::cout << "4. lz4 compress, without concat, with prime frame"
                << std::endl;
      std::cout << "5. zlib compress, with concat, without prime frame "
                << std::endl;
      std::cout << "6. zlib compress, with concat, with prime frame"
                << std::endl;
      std::cout << "7. zlib compress, without concat, without prime frame"
                << std::endl;
      std::cout << "8. zlib compress, without concat, with prime frame"
                << std::endl;
      std::cout << "9. zstd compress, with concat, without prime frame "
                << std::endl;
      std::cout << "10. zstd compress, with concat, with prime frame"
                << std::endl;
      std::cout << "11. zstd compress, without concat, without prime frame"
                << std::endl;
      std::cout << "12. zstd compress, without concat, with prime frame"
                << std::endl;
      std::cout << "13. zstd gray compress, with concat, without prime frame "
                << std::endl;
      std::cout << "14. zstd gray compress, with concat, with prime frame"
                << std::endl;
      std::cout << "15. zstd gray compress, without concat, without prime frame"
                << std::endl;
      std::cout << "16. zstd gray compress, without concat, with prime frame"
                << std::endl;
      std::cout << "Choose: ";

      try {
        // Получаем ввод пользователя
        if (!(std::cin >> choice)) {
          // Если ввод некорректный (не число)
          std::cin.clear(); // Сбрасываем флаг ошибки
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(),
                          '\n'); // Очищаем буфер
          std::cout << "Error enter number between 1...16" << std::endl;
          continue;
        }

        boost::asio::write(socket,
                           boost::asio::buffer(&choice, sizeof(choice)));
        // Обрабатываем выбор
        switch (choice) {
        case 1:
          lz4_concat_noprime(socket, cam1, cam2);
          break;

        case 2:
          lz4_concat_prime(socket, cam1, cam2);
          break;

        case 3:
          lz4_noconcat_noprime(socket, cam1, cam2);
          break;

        case 4:
          lz4_noconcat_prime(socket, cam1, cam2);
          break;

        case 5:
          zlib_concat_noprime(socket, cam1, cam2);
          break;

        case 6:
          zlib_concat_prime(socket, cam1, cam2);
          break;

        case 7:
          zlib_noconcat_noprime(socket, cam1, cam2);
          break;

        case 8:
          zlib_noconcat_prime(socket, cam1, cam2);
          break;

        case 9:
          zstd_concat_noprime(socket, cam1, cam2);
          break;

        case 10:
          zstd_concat_prime(socket, cam1, cam2);
          break;

        case 11:
          zstd_noconcat_noprime(socket, cam1, cam2);
          break;

        case 12:
          zstd_noconcat_prime(socket, cam1, cam2);
          break;

        case 13:
          zstd_gray_concat_noprime(socket, cam1, cam2);
          break;

        case 14:
          zstd_gray_concat_prime(socket, cam1, cam2);
          break;

        case 15:
          zstd_gray_noconcat_noprime(socket, cam1, cam2);
          break;

        case 16:
          zstd_gray_noconcat_prime(socket, cam1, cam2);
          break;

        default:
          std::cout << "Error enter number between 1...16" << std::endl;
        }
      } catch (const std::exception &ex) {
        std::cerr << "Connection Error: " << ex.what() << std::endl;
        std::cerr << "Reconnection.. " << std::endl;
        cv::destroyAllWindows();
      }
    } catch (const std::exception &ex) {
      std::cerr << "Connection Error: " << ex.what() << std::endl;
      std::cerr << "Reconnection.. " << std::endl;
      cv::destroyAllWindows();
    }
  }
  return 0;
}
