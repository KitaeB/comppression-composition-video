#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using boost::asio::ip::tcp;

const int PORT = 12345;

void send_image(tcp::socket& socket, const cv::Mat& image) {
    std::vector<uchar> buf;
    cv::imencode(".png", image, buf);  // Сжимаем изображение в формат JPG

    // Отправляем размер изображения
    std::string image_size = std::to_string(buf.size()) + "\n";
    boost::asio::write(socket, boost::asio::buffer(image_size));
    // Отправляем само изображение
    boost::asio::write(socket, boost::asio::buffer(buf));
}

int main() {
    try {
        boost::asio::io_context io_context;
        tcp::resolver resolver(io_context);
        tcp::socket socket(io_context);

        // Разрешаем хост и порт
        auto endpoints = resolver.resolve("127.0.0.1", std::to_string(PORT));
        boost::asio::connect(socket, endpoints);

        std::cout << "Connected to server.\n";

        // Загружаем изображение для отправки
        cv::Mat image = cv::imread("image.png");
        if (image.empty()) {
            std::cerr << "Failed to read image!" << std::endl;
        }

        // Отправляем изображение на сервер
        send_image(socket, image);
        std::cout << "Image sent to server.\n";

        // Получаем обработанное изображение
        std::vector<char> buffer;
        boost::asio::streambuf streambuf;
        boost::asio::read_until(socket, streambuf, "\n");
        std::istream is(&streambuf);
        std::string image_size_str;
        std::getline(is, image_size_str);
        int image_size = std::stoi(image_size_str);

        buffer.resize(image_size);
        boost::asio::read(socket, boost::asio::buffer(buffer), boost::asio::transfer_exactly(image_size));

        // Декодируем полученное изображение
        cv::Mat received_image = cv::imdecode(buffer, cv::IMREAD_COLOR);
        if (received_image.empty()) {
            std::cerr << "Failed to decode received image!" << std::endl;
        }

        // Показываем полученное обработанное изображение
        cv::imshow("Received Processed Image", received_image);
        cv::waitKey(0);

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    while (true)
    {

    }
    return 0;
}
