
#include "decomression.h"
#include "client.h"

#pragma region lz4

void lz4_concat_noprime(tcp::socket &socket) {
    int rows, cols, type;
    cv::Mat frame;
    std::vector<char> compressed_data, uncompressed_data;
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;

    while (true)
    {
        t0 = std::chrono::high_resolution_clock::now(); // До получения данных
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

        t1 = std::chrono::high_resolution_clock::now(); // После получения данных

        frame = cv::Mat(rows, cols, type);
        if (lz4_decompress(compressed_data, uncompressed_data, uncompressed_size))
        {
            frame = convertFromCleanDataChar(uncompressed_data, frame.rows, frame.cols, frame.type());
        }
        t2 = std::chrono::high_resolution_clock::now(); // После сжатия
        cv::imshow("webcam", frame);

        t3 = std::chrono::high_resolution_clock::now(); // После отображения

        std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                  << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                  << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                  << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                  << " uncompressed data: " << uncompressed_size
                  << " compressed data: " << compressed_size
                  << " koef: " << static_cast<double>(uncompressed_size) / static_cast<double>(compressed_size)
                  << std::endl;
        // Обязательно waitKey
        if (cv::waitKey(1) == 27)
        { // Нажал ESC
            break;
        }
    }
}

void lz4_concat_prime(tcp::socket &socket) {
    int rows, cols, type;
    uint16_t currentFrame;
    cv::Mat frame, prevFrame;
    std::vector<char> compressed_data, uncompressed_data;
    // Объявим временные метки
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4;

    while (true)
    {
        t0 = std::chrono::high_resolution_clock::now(); // До получения данных
        // Читаем метаданные
        boost::asio::read(socket, boost::asio::buffer(&rows, sizeof(rows)));
        boost::asio::read(socket, boost::asio::buffer(&cols, sizeof(cols)));
        boost::asio::read(socket, boost::asio::buffer(&type, sizeof(type)));
        boost::asio::read(socket, boost::asio::buffer(&currentFrame, sizeof(currentFrame)));

        int compressed_size;
        boost::asio::read(socket, boost::asio::buffer(&compressed_size, sizeof(compressed_size)));

        int uncompressed_size;
        boost::asio::read(socket, boost::asio::buffer(&uncompressed_size, sizeof(uncompressed_size)));

        // Читаем данные кадра
        compressed_data.resize(compressed_size);
        boost::asio::read(socket, boost::asio::buffer(compressed_data));

        t1 = std::chrono::high_resolution_clock::now(); // После получения данных

        frame = cv::Mat(rows, cols, type);
        if (lz4_decompress(compressed_data, uncompressed_data, uncompressed_size))
        {
            frame = convertFromCleanDataChar(uncompressed_data, frame.rows, frame.cols, frame.type());
            if (currentFrame != 0)
                frame = frameAddiiton(frame, prevFrame);
            prevFrame = frame.clone();
        }
        t2 = std::chrono::high_resolution_clock::now(); // После сжатия
        cv::imshow("webcam", frame);


        t3 = std::chrono::high_resolution_clock::now(); // После отображения

        std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                  << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                  << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                  << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                  << " uncompressed data: " << uncompressed_size
                  << " compressed data: " << compressed_size
                  << " koef: " << static_cast<double>(uncompressed_size) / static_cast<double>(compressed_size)
                  << std::endl;
        // Обязательно waitKey
        if (cv::waitKey(1) == 27)
        { // Нажал ESC
            break;
        }
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

    while (true)
    {
        t0 = std::chrono::high_resolution_clock::now(); // До получения данных
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

        t1 = std::chrono::high_resolution_clock::now(); // После получения данных

        frame = cv::Mat(rows, cols, type);
        if (zlib_decompress(compressed_data, uncompressed_data, uncompressed_size) == 0)
        {
            frame = convertFromCleanDataBytef(uncompressed_data, frame.rows, frame.cols, frame.type());
        }
        t2 = std::chrono::high_resolution_clock::now(); // После сжатия
        cv::imshow("webcam", frame);

        t3 = std::chrono::high_resolution_clock::now(); // После отображения

        std::cout << " get data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                  << " decompress data: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
                  << " get image: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
                  << " FPS: " << 1000 / std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count()
                  << " uncompressed data: " << uncompressed_size
                  << " compressed data: " << compressed_size
                  << " koef: " << static_cast<double>(uncompressed_size) / static_cast<double>(compressed_size)
                  << std::endl;
        // Обязательно waitKey
        if (cv::waitKey(1) == 27)
        { // Нажал ESC
            break;
        }
    }
}

#pragma endregion