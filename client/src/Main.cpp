#include "decomression.h"
#include "client.h"

const int PORT = 9090;
const char *HOST = "127.0.0.1";

int main()
{
    while (true)
    {
        try
        {
            boost::asio::io_context io_context;
            tcp::resolver resolver(io_context);
            tcp::socket socket(io_context);

            socket.connect(tcp::endpoint(boost::asio::ip::make_address(HOST), PORT));
            int method = 0;

            boost::asio::read(socket, boost::asio::buffer(&method, sizeof(method)));
            std::cout << "method: " << method << std::endl;
            switch (method)
            {
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
                break;

            case 5:
                zlib_concat_noprime(socket);
                break;

            case 6:
                break;

            case 7:
                zlib_noconcat_noprime(socket); 
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
        }
        catch (const std::exception &ex)
        {
            std::cerr << "Ошибка: " << ex.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return 0;
}
