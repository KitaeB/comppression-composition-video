#include "client.h"
#include <boost/asio/read.hpp>
#include <iostream>
#include <ostream>
#include <thread>

const int PORT = 9090;
std::string HOST;

int main() {
    while (true) {
        try {
            std::cout << "Enter HOST (default localhost): ";
            std::getline(std::cin, HOST);
            if (HOST.empty()) HOST = "127.0.0.1";
            std::cout << "    Your host is " << HOST << std::endl;

            boost::asio::io_context io_context;
            tcp::resolver resolver(io_context);
            tcp::socket socket(io_context);
            boost::system::error_code ec;

            socket.connect(tcp::endpoint(boost::asio::ip::make_address(HOST.c_str()), PORT), ec);
            int method = 0;
            if (ec) {
                std::cerr << "Socket error: " << ec << std::endl;
                continue;
            } else {
                std::cout << "Connection complete." << std::endl;
            }
            
            boost::asio::read(socket, boost::asio::buffer(&method, sizeof(method)));
            std::cout << "method: " << method << std::endl;
            switch (method) {
                case 1:
                    lz4_concat_noprime(socket);
                    break;

                case 2:
                    lz4_concat_prime(socket);
                    break;

                case 3:
                    lz4_noconcat_noprime(socket);
                    break;
                case 4:
                    lz4_noconcat_prime(socket);
                    break;

                case 5:
                    zlib_concat_noprime(socket);
                    break;

                case 6:
                    zlib_concat_prime(socket);
                    break;

                case 7:
                    zlib_noconcat_noprime(socket);
                    break;

                case 8:
                    zlib_noconcat_prime(socket);
                    break;

                case 9:
                    zstd_concat_noprime(socket);
                    break;

                case 10:
                    zstd_concat_prime(socket);
                    break;

                case 11:
                    zstd_noconcat_noprime(socket);
                    break;

                case 12:
                    zstd_noconcat_prime(socket);
                    break;

                case 13:
                    zstd_gray_concat_noprime(socket);
                    break;

                case 14:
                    zstd_gray_concat_prime(socket);
                    break;

                case 15:
                    zstd_gray_noconcat_noprime(socket);
                    break;

                case 16:
                    zstd_gray_noconcat_prime(socket);
                    break;

                default:
                    std::cout << "Error enter number between 1...16" << std::endl;
            }
        } catch (const std::exception &ex) {
            std::cerr << "Ошибка: " << ex.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return 0;
}
