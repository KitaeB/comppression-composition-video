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
    
    //Цикл для реалиализации переподключения для сервера, в случае ошибки
    while (true)
    {  
        //Инициализируем новое подключение
        server.NewConnect();
        
        //Получаем сокет нашего соединения
        tcp::socket& socket (server.GetSocket());

        //Объявляем выбор метода
        uint8_t method;
        std::cout <<    "Choose method: \n" << 
                        "Method 1: \n" <<
                        "Method 2: \n" 
                        << std::endl;
        
        std::cin >> method;
        //Обработка выбора метода
        switch (method){
            case 1: 
                ;
                break;
            case 2:
                ;
                break;
            default:
                std::cout << "Invalid choose" << std::endl;
                break;
        }

        try
        {
            // основной цикл
            while (true)
            {
                //Читаем данные с камер
                cap1 >> frame1;
                cap2 >> frame2;

                //Проверям, что кадры не пустые
                if (frame1.empty() || frame2.empty())
                {
                    std::cerr << "empty frame" << std::endl;
                    return -1;
                }

                //Изменяем размер и производим соединение кадров
                cv::resize(frame2, frame2, frame1.size());
                cv::hconcat(frame1, frame2, frame);

                    // Объявим метаданные передаваеммого кадра
                    int rows = frame.rows;
                    int cols = frame.cols;
                    int type = frame.type();
                    // Передаём метаданные по сокету
                    boost::asio::write(socket, boost::asio::buffer(&rows, sizeof(rows)));
                    boost::asio::write(socket, boost::asio::buffer(&cols, sizeof(cols)));
                    boost::asio::write(socket, boost::asio::buffer(&type, sizeof(type)));

                    cv::Mat endFrame = frame.clone();
                    // Отправка данных клиенту
                    boost::asio::write(socket, boost::asio::buffer(endFrame.data, (endFrame.total() * endFrame.elemSize())));

                    cv::imshow("source", endFrame);
                    prevFrame = frame.clone();

                if (cv::waitKey(1) == 27)
                { // Esc key to stop
                    break;
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