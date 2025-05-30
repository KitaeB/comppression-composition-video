#include <boost/asio.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "server.h"

using boost::asio::ip::tcp;

int main() {
  // Создаём сервер
  TcpServer server(9090);

  while (true) {
  // Подключаем видео
  //CameraState cam1{cv::VideoCapture("video/file_1.mp4")};
  //CameraState cam2{cv::VideoCapture("video/file_2.mp4")};

  // Подключаем камеры
  CameraState cam1{cv::VideoCapture(0, cv::CAP_DSHOW)};
  CameraState cam2{cv::VideoCapture(1, cv::CAP_DSHOW)};

  int choice = 0;
  // std::vector<Bytef> compressed_data;
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
      std::cout << "9. aom compress, with concat, without prime frame "
                << std::endl;
      std::cout << "10. aom compress, with concat, with prime frame"
                << std::endl;
      std::cout << "11. aom compress, without concat, without prime frame"
                << std::endl;
      std::cout << "12. aom compress, without concat, with prime frame"
                << std::endl;
      std::cout << "13. ?? compress, with concat, without prime frame "
                << std::endl;
      std::cout << "14. ?? compress, with concat, with prime frame"
                << std::endl;
      std::cout << "15. ?? compress, without concat, without prime frame"
                << std::endl;
      std::cout << "16. ?? compress, without concat, with prime frame"
                << std::endl;
      std::cout << "Choose: ";

      try {
        // Получаем ввод пользователя
        if (!(std::cin >> choice)) {
          // Если ввод некорректный (не число)
          std::cin.clear(); // Сбрасываем флаг ошибки
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(),
                          '\n'); // Очищаем буфер
          std::cout << "Error enter number between 1...4" << std::endl;
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
          break;

        case 7:
          break;

        case 8:
          break;

        case 9:
          break;

        case 10:
          break;

        case 11:
          break;

        case 12:
          break;

        case 13:
          break;

        case 14:
          break;

        case 15:
          break;

        case 16:
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