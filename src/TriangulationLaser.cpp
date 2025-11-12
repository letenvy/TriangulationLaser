#include "TriangulationLaser.hpp"

#include <iostream>
#include <thread>
#include <chrono>

namespace{
    bool parseRiftekResponse(const std::vector<uint8_t>& raw, std::vector<uint8_t>& outData) {
        if (raw.empty()) return false;

        outData.clear();
        outData.reserve(raw.size() / 2);

        for (size_t i = 0; i < raw.size(); i += 2) {
            if (i + 1 >= raw.size()) {
                std::cerr << "[Laser] Malformed Riftek response: incomplete byte pair\n";
                return false;
            }
            uint8_t low = raw[i] & 0x0F;
            uint8_t high = (raw[i + 1] & 0x0F) << 4;
            outData.push_back(low | high);
        }
        return true;
    }

    std::vector<uint8_t> buildRiftekRequest(uint8_t address, uint8_t cmd, const std::vector<uint8_t>& msg = {}) {
        std::vector<uint8_t> request;
        request.reserve(2 + msg.size() * 2);

        request.push_back(static_cast<uint8_t>(address & 0x7F));// 0 + ADR
        request.push_back(static_cast<uint8_t>(0x80 | cmd));    // 1 + CMD

        for (uint8_t b : msg) {
            uint8_t low = b & 0x0F;
            uint8_t high = (b >> 4) & 0x0F;
            request.push_back(static_cast<uint8_t>(0x80 | low));
            request.push_back(static_cast<uint8_t>(0x80 | high));
        }
        return request;
    }
}

TriangulationLaser::TriangulationLaser(const std::string& portName,
                                       int baudrate, char parity,
                                       int dataBits, int stopBits,
                                       double sensorRangeMm, double baseRangeMm)
    : portName_(portName)
    , baudrate_(baudrate)
    , parity_(parity)
    , dataBits_(dataBits)
    , stopBits_(stopBits)
    , sensorRangeMm_(sensorRangeMm)
    , minDistanceMm_(baseRangeMm)
    , serial_connected_(false)
    , mode_(LaserMode::Unknown)
{
    portName_ = normalizePortName(portName_);
}

TriangulationLaser::~TriangulationLaser(){
    disconnect();
}

bool TriangulationLaser::connect() {
    if (serial_connected_) return true;

    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_);
        if (!setupSerialPort(*serial_port_)) {
            return false;
        }
        serial_connected_ = true;
        mode_ = LaserMode::Riftek;
        std::cout << "[Laser] Connected in RIFTEK mode to " << portName_ << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[Laser ERROR] connect: " << e.what() << std::endl;
        return false;
    }
}

void TriangulationLaser::disconnect() {
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
        serial_connected_ = false;
    }
    mode_ = LaserMode::Unknown;
    std::cout << "[Laser] Disconnected\n";
}

bool TriangulationLaser::isConnected() const {
    return serial_connected_;
}

std::string TriangulationLaser::normalizePortName(const std::string& port) const {
    if (!port.empty() && port.find("COM") == 0 && port.size() > 4)
        return "\\\\.\\" + port;
    return port;
}

bool TriangulationLaser::setupSerialPort(boost::asio::serial_port& serial) {
    std::string port = normalizePortName(portName_);
    serial.open(port);

    serial.set_option(boost::asio::serial_port::baud_rate(baudrate_));
    serial.set_option(boost::asio::serial_port::character_size(dataBits_));

    auto par = boost::asio::serial_port::parity::none;
    if (parity_ == 'E') par = boost::asio::serial_port::parity::even;
    else if (parity_ == 'O') par = boost::asio::serial_port::parity::odd;
    serial.set_option(boost::asio::serial_port::parity(par));

    auto stop = boost::asio::serial_port_base::stop_bits::one;
    if (stopBits_ == 2) stop = boost::asio::serial_port_base::stop_bits::two;
    serial.set_option(boost::asio::serial_port_base::stop_bits(stop));

    return true;
}

double TriangulationLaser::convertRawToMm(uint16_t rawValue) const {
    if (rawValue == 0 || rawValue == 16384) {
        return 0.0;
    }
    return minDistanceMm_ + static_cast<double>(rawValue) * (sensorRangeMm_ - minDistanceMm_) / 16384.0;
}

bool TriangulationLaser::Riftek::sendRequest(const std::vector<uint8_t>& request) {
    if (!parent_.serial_connected_) {
        std::cerr << "[Laser] Not connected\n";
        return false;
    }

    boost::system::error_code ec;
    parent_.serial_port_->write_some(boost::asio::buffer(request), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

std::optional<std::vector<uint8_t>> TriangulationLaser::Riftek::listenAnswer(int timeout_ms,size_t max_size) {
    if (!parent_.serial_connected_) {
        std::cerr << "[Laser] Not connected\n";
        return std::nullopt;
    }

    std::vector<uint8_t> response(max_size);

    boost::asio::deadline_timer timer(parent_.io_);
    timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));

    std::promise<std::pair<std::size_t, boost::system::error_code>> result_promise;
    auto result_future = result_promise.get_future();

    bool timed_out = false;

    parent_.serial_port_->async_read_some(boost::asio::buffer(response),
        [&result_promise, &timed_out](const boost::system::error_code& e, std::size_t bytes_transferred) {
            if (!timed_out) {
                result_promise.set_value({bytes_transferred, e});
            }
        });

    timer.async_wait([&timed_out, this](const boost::system::error_code& e) {
        if (!e) {
            timed_out = true;
            parent_.serial_port_->cancel();
        }
    });

    auto status = result_future.wait_for(std::chrono::milliseconds(timeout_ms + 100));
    if (status == std::future_status::timeout || timed_out) {
        std::cerr << "[Laser] Read timeout\n";
        return std::nullopt;
    }

    auto [bytes, ec] = result_future.get();
    if (ec) {
        std::cerr << "[Laser] Read error: " << ec.message() << std::endl;
        return std::nullopt;
    }

    response.resize(bytes);
    return response;
}

std::optional<uint16_t> TriangulationLaser::Riftek::requestResult(){
    auto req = buildRiftekRequest(parent_.riftek_address_, 0x06);
    if (!sendRequest(req)) return std::nullopt;

    auto resp = listenAnswer();
    if (!resp) return std::nullopt;

    std::vector<uint8_t> data;
    if (!parseRiftekResponse(*resp, data) || data.size() < 2) {
        std::cerr << "[Laser] Invalid response for result request\n";
        return std::nullopt;
    }

    return (static_cast<uint16_t>(data[1]) << 8) | data[0];
}

std::optional<std::vector<uint8_t>> TriangulationLaser::Riftek::requestID(){
    auto req = buildRiftekRequest(parent_.riftek_address_, 0x01);
    if (!sendRequest(req)) return std::nullopt;

    auto resp = listenAnswer(2000, 20);
    if (!resp) return std::nullopt;

    std::vector<uint8_t> data;
    if (!parseRiftekResponse(*resp, data) || data.size() < 8) {
        std::cerr << "[Laser] Invalid response for ID request\n";
        return std::nullopt;
    }

    return data;
}

bool TriangulationLaser::Riftek::startStream() {
    auto req = buildRiftekRequest(parent_.riftek_address_, 0x07);
    return sendRequest(req);
}

bool TriangulationLaser::Riftek::stopStream() {
    auto req = buildRiftekRequest(parent_.riftek_address_, 0x08);
    return sendRequest(req);
}

bool TriangulationLaser::Riftek::writeParameter(uint8_t paramCode, uint8_t value) {
    auto req = buildRiftekRequest(parent_.riftek_address_, 0x03, {paramCode, value});
    return sendRequest(req);
}

std::optional<uint8_t> TriangulationLaser::Riftek::readParameter(uint8_t paramCode) {
    auto req = buildRiftekRequest(parent_.riftek_address_, 0x02, {paramCode});
    if (!sendRequest(req)) return std::nullopt;

    auto resp = listenAnswer();
    if (!resp) return std::nullopt;

    std::vector<uint8_t> data;
    if (!parseRiftekResponse(*resp, data) || data.size() < 1) {
        std::cerr << "[Laser] Invalid response for parameter read\n";
        return std::nullopt;
    }

    return data[0];
}