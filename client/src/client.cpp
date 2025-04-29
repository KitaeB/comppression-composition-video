#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using boost::asio::ip::tcp;

const int PORT = 9090;
const char *HOST = "127.0.0.1";
int main()
{

    int rows, cols, type;
    cv::Mat frame;
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
                // Читаем метаданные
                boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
                boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
                boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));

                if (rows <= 0 || cols <= 0)
                {
                    std::cerr << "Ошибка получения размера кадра!" << std::endl;
                    break;
                }

                frame = cv::Mat(rows, cols, type);

                // Читаем данные кадра
                boost::asio::read(socket, boost::asio::buffer(frame.data, frame.total() * frame.elemSize()));

                cv::imshow("webcam", frame);

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
