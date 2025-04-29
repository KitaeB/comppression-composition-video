#include "server.h"

TcpServer::TcpServer(const uint16_t port)
    : socket(io_context),
      acceptor(io_context, tcp::endpoint(tcp::v4(), port)),
      PORT(port)
{}

void TcpServer::NewConnect()
{
    //Закрываем старое соединение, если есть
    if (socket.is_open())
    {
        boost::system::error_code ec;
        socket.close(ec);
        if(ec)
        {
            std::cerr << "Error connection close" << std:: endl;
        }
    }

    //Открываем новое соединение
    std::cout << "Waiting new connect.." << std::endl;
    acceptor.accept(socket);
    if (!socket.is_open())
    {
        std::cerr << "failed to open socket" << std::endl;
    }
}

tcp::socket& TcpServer::GetSocket()
{
    return socket;
}
bool TcpServer::TestConnect()
{
    return socket.is_open();
}