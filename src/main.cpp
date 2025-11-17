#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <boost/asio.hpp>
#include <sstream>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

#include "TriangulationLaser.hpp"

using boost::asio::ip::tcp;
using json=nlohmann::json;

class LaserTCPServer {
public:
    LaserTCPServer(boost::asio::io_context& io_context, short port,TriangulationLaser& laser)
        : acceptor_(io_context, tcp::endpoint(tcp::v4(), port)),
        laser_ref_(laser)
        {

        auto now=std::chrono::system_clock::now();
        auto time_t=std::chrono::system_clock::to_time_t(now);
        auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()%1000;
        
        std::stringstream ss;
        ss<<std::put_time(std::localtime(&time_t),"scan_%Y%m%d_%H%M%S");
        ss<<"_"<<std::setfill('0')<<std::setw(3)<<ms<<".json";

        std::string filename=ss.str();

        log_file_.open(filename);
        log_file_<<"[\n";
        log_file_.flush();
        do_accept();
    }
    ~LaserTCPServer(){
        if(log_file_.is_open()){
            log_file_<<"\n]";
            log_file_.close();
        }
    }

private:
    tcp::acceptor acceptor_;
    std::ofstream log_file_;
    bool first_entry_=true;
    long long prev_timestamp_=0;
    TriangulationLaser& laser_ref_;

    void do_accept() {
        acceptor_.async_accept([this](boost::system::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<Session>(std::move(socket),log_file_,first_entry_,prev_timestamp_,laser_ref_)->start();
            }
            do_accept();
        });
    }

    class Session : public std::enable_shared_from_this<Session> {
    public:
        Session(tcp::socket socket,std::ofstream& log_file,bool& first_emtry,long long& prev_timestamp, TriangulationLaser& laser)
            : socket_(std::move(socket)),
            log_file_ref_(log_file),
            first_entry_ref_(first_emtry),
            prev_timestamp_ref_(prev_timestamp),
            laser_ref_(laser) {}

        void start() {
            do_read();
        }

    private:
        tcp::socket socket_;
        std::array<char, 1024> buffer_;
        std::ofstream& log_file_ref_;
        bool& first_entry_ref_;
        long long& prev_timestamp_ref_;
        TriangulationLaser& laser_ref_;
        

        void do_read() {
            auto self = shared_from_this();
            socket_.async_read_some(boost::asio::buffer(buffer_),
                [this, self](boost::system::error_code ec, std::size_t length) {
                    if (!ec) {
                        std::string request(buffer_.data(), length);
                        std::cout << "Received: " << request;

                        double x=0,y=0,z=0;
                        long long t=0;
                        
                        std::istringstream iss(request);
                        std::string temp;
                        iss>>temp;
                        iss>>temp;
                        x=std::stod(temp.substr(2));
                        iss>>temp;
                        y=std::stod(temp.substr(2));
                        iss>>temp;
                        z=std::stod(temp.substr(2));
                        iss>>temp;
                        t=std::stod(temp.substr(2));
                        iss>>temp;

                        double polling_freq_hz=1000;
                        if(prev_timestamp_ref_!=0&&t>prev_timestamp_ref_){
                            double delta_ms=t-prev_timestamp_ref_;
                            polling_freq_hz=1000.0/delta_ms;
                        }else{
                            polling_freq_hz=0;
                        }
                        prev_timestamp_ref_=t;
                        
                        auto rf=laser_ref_.riftek();

                        uint16_t raw_value=0;
                        double distance=0.0;
                        
                        if(auto raw_opt=rf.requestResult()){
                            raw_value=*raw_opt;
                            distance=laser_ref_.convertRawToMm(raw_value);
                        }else{
                            std::cerr<<"[Laser] Failed to read result!"<<std::endl;
                            distance=0.0;
                        }

                        json j;
                        j["timestamp_ms"] = t;
                        j["robot_x_mm"] = x;
                        j["robot_y_mm"] = y;
                        j["robot_z_mm"] = z;
                        j["laser_distance_mm"] = distance;
                        j["raw_value"] = raw_value;

                        std::stringstream hex_ss;
                        hex_ss<<"0x"<<std::hex<<raw_value;

                        j["hex_value"] =hex_ss.str();

                        j["polling_frequency_hz"] = polling_freq_hz;

                        
                        if(!first_entry_ref_){
                            log_file_ref_<<",\n";
                        }
                        log_file_ref_<<j.dump(2);
                        log_file_ref_.flush();
                        first_entry_ref_=false;

                        std::string response = "DIST: "+std::to_string(distance);
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
                        do_read();
                    }
                });
        }
    };
};

int main() {
    try {
        TriangulationLaser laser("COM5", 9600, 'E', 8, 1, 500.0, 125.0);

        if(!laser.connect()){
            std::cerr<<"Failed to connect to laser!"<<std::endl;
            return 1;
        }

        boost::asio::io_context io_context;
        LaserTCPServer server(io_context, 12345,laser);
        std::cout << "TCP Server listening on port 12345...\n";
        io_context.run();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}