#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#include "decomression.h"

using boost::asio::ip::tcp;

const int PORT = 9090;
const char *HOST = "127.0.0.1";
int main()
{

    int rows, cols, type;
    cv::Mat frame;
    std::vector<char> compressed_data, uncompressed_data;
    //Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;
    while (true)
    {
        try
        {
            boost::asio::io_context io_context;
            tcp::resolver resolver(io_context);
            tcp::socket socket(io_context);

            socket.connect(tcp::endpoint(boost::asio::ip::make_address(HOST), PORT));

            while (true)
            {
                t0 = std::chrono::high_resolution_clock::now();         //До получения данных
                // Читаем метаданные
                boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
                
                int compressed_size;
                boost::asio::read(socket, boost::asio::buffer(&compressed_size, sizeof(compressed_size)));
                
                int uncompressed_size;
                boost::asio::read(socket, boost::asio::buffer(&uncompressed_size, sizeof(uncompressed_size)));

                // Читаем данные кадра
                compressed_data.resize(compressed_size);
                boost::asio::read(socket, boost::asio::buffer(compressed_data));

                t1 = std::chrono::high_resolution_clock::now();         //После получения данных

                frame = cv::Mat(rows, cols, type);
                if(lz4_decompress(compressed_data, uncompressed_data, uncompressed_size)){
                        frame = convertFromCleanData(uncompressed_data, frame.rows, frame.cols, frame.type());
                    }
                t2 = std::chrono::high_resolution_clock::now();         //После сжатия
                cv::imshow("webcam", frame);

                t3 = std::chrono::high_resolution_clock::now();         //После отображения

                std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                          << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                          << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                          << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                          << " uncompressed data: " << uncompressed_size
                          << " compressed data: " << compressed_size
                              << " koef: " << static_cast<double>(uncompressed_size)/static_cast<double>(compressed_size)
                          << std::endl;
                // Обязательно waitKey
                if (cv::waitKey(1) == 27)
                { // Нажал ESC
                    break;
                }
            }
        }
        catch (const std::exception &ex)
        {
            std::cerr << "Ошибка: " << ex.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return 0;
    }
}
