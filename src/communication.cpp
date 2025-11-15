#include <iostream>
#include <string>
#include <array>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class LaserTCPServer {
public:
    LaserTCPServer(boost::asio::io_context& io_context, short port)
        : acceptor_(io_context, tcp::endpoint(tcp::v4(), port)) {
        do_accept();
    }

private:
    tcp::acceptor acceptor_;

    void do_accept() {
        acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<Session>(std::move(socket))->start();
            }
            do_accept();
        });
    }

    class Session : public std::enable_shared_from_this<Session> {
    public:
        Session(tcp::socket socket) : socket_(std::move(socket)) {}

        void start() {
            do_read();
        }

    private:
        tcp::socket socket_;
        std::array<char, 1024> buffer_;

        void do_read() {
            auto self = shared_from_this();
            socket_.async_read_some(boost::asio::buffer(buffer_),
                [this, self](boost::system::error_code ec, std::size_t length) {
                    if (!ec) {
                        std::string request(buffer_.data(), length);
                        std::cout << "Received: " << request;

                        std::string response = "DUMMY";
                        do_write(response);
                    }
                });
        }

        void do_write(const std::string& msg) {
            auto self = shared_from_this();
            boost::asio::async_write(socket_, boost::asio::buffer(msg, msg.length()),
                [this, self, msg](boost::system::error_code ec, std::size_t) {
                    if (!ec) {
                        std::cout << " -> Sent: " << msg << std::endl;
                    }
                });
        }
    };
};

int main() {
    try {
        boost::asio::io_context io_context;
        LaserTCPServer server(io_context, 12345);
        std::cout << "TCP Server listening on port 12345...\n";
        io_context.run();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}