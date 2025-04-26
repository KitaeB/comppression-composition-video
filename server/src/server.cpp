#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using boost::asio::ip::tcp;

const int PORT = 12345;  // Порт для соединения

// Функция для получения изображения от клиента
std::vector<char> receive_image(tcp::socket& socket) {
    std::vector<char> buffer;
    boost::asio::streambuf streambuf;

    boost::asio::read_until(socket, streambuf, "\n");  // Чтение до конца сообщения
    std::istream is(&streambuf);
    std::string image_size_str;
    std::getline(is, image_size_str);  // Получаем размер изображения
    int image_size = std::stoi(image_size_str);  // Преобразуем в число

    buffer.resize(image_size);
    boost::asio::read(socket, boost::asio::buffer(buffer), boost::asio::transfer_exactly(image_size));

    return buffer;
}

// Функция для отправки изображения обратно клиенту
void send_image(tcp::socket& socket, const cv::Mat& image) {
    std::vector<uchar> buf;
    cv::imencode(".jpg", image, buf);  // Сжимаем изображение в формат JPG

    // Отправляем размер изображения
    std::string image_size = std::to_string(buf.size()) + "\n";
    boost::asio::write(socket, boost::asio::buffer(image_size));
    // Отправляем само изображение
    boost::asio::write(socket, boost::asio::buffer(buf));
}

int main() {
    try {
        boost::asio::io_context io_context;
        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), PORT));

        std::cout << "Server started, waiting for connection...\n";
        
        // Принимаем соединение от клиента
        tcp::socket socket(io_context);
        acceptor.accept(socket);
        std::cout << "Client connected!\n";

        // Получаем изображение от клиента
        auto received_image_data = receive_image(socket);

        // Преобразуем данные в формат изображения
        cv::Mat received_image = cv::imdecode(received_image_data, cv::IMREAD_COLOR);
        if (received_image.empty()) {
            std::cerr << "Failed to decode image!" << std::endl;
        }

        std::cout << "Received image: " << received_image.cols << "x" << received_image.rows << std::endl;

        // Показываем полученное обработанное изображение
        cv::imshow("Received Processed Image", received_image);

        // Обрабатываем изображение (например, преобразуем в серый цвет)
        cv::Mat processed_image;
        cv::cvtColor(received_image, processed_image, cv::COLOR_BGR2GRAY);

        // Отправляем обратно обработанное изображение
        send_image(socket, processed_image);

        std::cout << "Processed image sent back to client.\n";
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    while (true)
    {
        
    }
    return 0;
}
