#pragma once 

#include <boost/asio.hpp>
#include <iostream>

using boost::asio::ip::tcp;

class TcpServer 
{
public:
    TcpServer(const uint16_t port);
    void NewConnect();
    bool TestConnect();
    tcp::socket& GetSocket();

private:
    uint16_t PORT;
    boost::asio::io_context io_context;
    tcp::socket socket;
    tcp::acceptor acceptor;
};
