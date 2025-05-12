#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "compression.h"
#include "server.h"


using boost::asio::ip::tcp;

int main()
{
    // Подключаем камеры
    cv::VideoCapture cap1(0, cv::CAP_ANY);
    cv::VideoCapture cap2(1, cv::CAP_ANY);
    
    //Ожидание открытия камер
    if (!cap1.isOpened() || !cap2.isOpened())
    {
        std::cerr << "failed oppened cap" << std::endl;
        return -1;
    }

    //Создаём сервер
    TcpServer server(9090);

    // Объявляем фреймы
    cv::Mat frame1, frame2, frame, prevFrame;
    std::vector<char> compressed_data, uncompressed_data;
    uint16_t acceleration = 1;

    //Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;

    int choice = 0;
    //std::vector<Bytef> compressed_data;
    //Цикл для реалиализации переподключения для сервера, в случае ошибки
    while (true)
    {  
        //Инициализируем новое подключение
        server.NewConnect();
        
        //Получаем сокет нашего соединения
        tcp::socket& socket (server.GetSocket());

        // Выводим меню
        std::cout << "\n===  ===" << std::endl;
        std::cout << "1. " << std::endl;
        std::cout << "2. " << std::endl;
        std::cout << "3. " << std::endl;
        std::cout << "4. Exit" << std::endl;
        std::cout << "Choose: ";
        
        // Получаем ввод пользователя
        if(!(std::cin >> choice)) {
            // Если ввод некорректный (не число)
            std::cin.clear(); // Сбрасываем флаг ошибки
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Очищаем буфер
            std::cout << "Error enter number between 1...4" << std::endl;
            continue;
        }
         // Обрабатываем выбор
        switch(choice) {
            case 1:
                std::cout << "You choose 1" << std::endl;
                // Здесь код для опции 1
                break;
                
            case 2:
                std::cout << "You choose 2" << std::endl;
                // Здесь код для опции 2
                break;
                
            case 3:
                std::cout << "You choose 3" << std::endl;
                // Здесь код для опции 3
                break;
                
            case 4:
                std::cout << "Exit..." << std::endl;
                return 0;
                
            default:
                std::cout << "Error enter number between 1...4" << std::endl;
        }
        try
        {
            // основной цикл
            while (true)
            {
                t0 = std::chrono::high_resolution_clock::now();         //До получения картинки
                //Читаем данные с камер
                cap1 >> frame1;
                cap2 >> frame2;

                //Проверям, что кадры не пустые
                if (frame1.empty() || frame2.empty())
                {
                    std::cerr << "empty frame" << std::endl;
                    return -1;
                }
                //После получения
                t1 = std::chrono::high_resolution_clock::now();

                //Изменяем размер и производим соединение кадров
                cv::resize(frame2, frame2, frame1.size());
                cv::hconcat(frame1, frame2, frame);
                auto last_duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
                if (last_duration > 35 && acceleration < 100)
                    acceleration++;
                else if(last_duration < 25 && acceleration > 1)
                    acceleration--;
                t2 = std::chrono::high_resolution_clock::now();         //Изменение размера и объединение

                //Преобразуем данные в vector<char>
                uncompressed_data = convertToCleanData(frame);
                //Сожмём данные с zlib-default
                if (lz4_compress_fast(uncompressed_data, compressed_data, acceleration) > 0){
                    t3 = std::chrono::high_resolution_clock::now();     //После сжатия
                    // Объявим метаданные передаваеммого кадра
                    int rows = frame.rows;
                    int cols = frame.cols;
                    int type = frame.type();

                    //std::cout << "rows: " << rows << " Cols: " << cols << " Type: " << type << std::endl;

                    // Передаём метаданные по сокету
                    boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                    boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                    boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                    int compressed_size = compressed_data.size();
                    boost::asio::write(socket, boost::asio::buffer(&compressed_size, sizeof(compressed_size)));

                    int uncompressed_size = uncompressed_data.size();
                    boost::asio::write(socket, boost::asio::buffer(&uncompressed_size, sizeof(uncompressed_size)));

                    // Отправка данных клиенту
                    boost::asio::write(socket, boost::asio::buffer(compressed_data));

                    t4 = std::chrono::high_resolution_clock::now();         //После передачи
                    prevFrame = frame.clone();

                    std::cout << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                              << " convert image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                              << " compress: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                              << " send: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
                              << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t0).count()
                              << " acceleration: " << acceleration
                              << " uncompressed data: " << uncompressed_size
                              << " compressed data: " << compressed_size
                              << " koef: " << static_cast<double>(uncompressed_size)/static_cast<double>(compressed_size)
                              << std::endl;

                    cv::imshow("source", frame);

                if (cv::waitKey(1) == 27)
                { // Esc key to stop
                    break;
                }
                }
            }
        }
            catch (const std::exception &ex)
            {
                std::cerr << "Connection Error: " << ex.what() << std::endl;
                std::cerr << "Reconnection.. " << std::endl;
                cv::destroyAllWindows();
            }
    }
        cap1.release();
        cap2.release();
        cv::destroyAllWindows();
        return 0;
}