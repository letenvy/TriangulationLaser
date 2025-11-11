#include "TriangulationLaser.hpp"

namespace{
    std::vector<uint8_t> buildRiftekRequest(uint8_t address,uint8_t cmd,const std::vector<uint8_t>&msg={}){
        std::vector<uint8_t> request;
        request.reserve(2+msg.size()*2);

        request.push_back(static_cast<uint8_t>(address&0x7F));

        request.push_back(static_cast<uint8_t>(0x80|((cmd&0x0F)<<3)));

        for(uint8_t b:msg){
            uint8_t low=b & 0x0F;
            uint8_t high=(b>>4) & 0x0F;
            request.push_back(static_cast<uint8_t>(0x80|low));
            request.push_back(static_cast<uint8_t>(0x80|high));
        }
        return request;
    }
    bool parseRiftekResponse(const std::vector<uint8_t> & raw, std::vector<uint8_t> & outData){
        if(raw.empty())return false;

        outData.clear();
        outData.reserve(raw.size()/2);

        for(size_t i=0;i<raw.size();i+=2){
            if(i+1>=raw.size()){
                std::cerr<<"[Laser] Malformed Riftek response: incomplete byte pair"<<std::endl;
                return false;
            }
            uint8_t low=raw[i]&0x0F;
            uint8_t high=(raw[i+1]&0x0F)<<4;
            outData.push_back(low|high);
        }
        return true;
    }
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

void TriangulationLaser::handleError(const std::string& context) const {
    const char* err = modbus_strerror(errno);
    if (!err || std::strlen(err) == 0) {
        err = strerror(errno);
    }
    std::cerr << "[Laser ERROR] " << context << ": " << err << " (errno=" << errno << ")" << std::endl;
}

double TriangulationLaser::convertRawToMm(uint16_t rawValue) const {
    constexpr double FULL_SCALE = 16384.0; // 0x4000
    if (sensorRangeMm_ <= 0.0) {
        std::cerr << "[Laser] Invalid sensor range: " << sensorRangeMm_ << std::endl;
        return 0.0;
    }
    return static_cast<double>(rawValue) * sensorRangeMm_ / FULL_SCALE;
}

TriangulationLaser::TriangulationLaser(const std::string& portName,
                                       int baudrate, char parity,
                                       int dataBits, int stopBits,
                                       int slaveId, double sensorRangeMm)
    : portName_(portName)
    , baudrate_(baudrate)
    , parity_(parity)
    , dataBits_(dataBits)
    , stopBits_(stopBits)
    , slaveId_(slaveId)
    , sensorRangeMm_(sensorRangeMm)
    , ctx_(nullptr)
    , serial_connected_(false)
    , modbus_connected_(false)
    , mode_(LaserMode::Unknown)
    , riftek_address_(1)
    , last_riftek_cnt_(0)
    , riftek_streaming_(false)
{
    portName_ = normalizePortName(portName_);
}

TriangulationLaser::~TriangulationLaser() {
    disconnect();
}

bool TriangulationLaser::connect() {
    if (serial_connected_) return true;

    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_);
        setupSerialPort(*serial_port_);
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
    if (riftek_streaming_) {
        std::cout << "[Laser] Stopping stream before disconnect . . ."<<std::endl;
        riftek_stopStream();
    }

    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
        serial_connected_ = false;
    }

    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
        modbus_connected_ = false;
    }

    mode_ = LaserMode::Unknown;
    std::cout << "[Laser] Disconnected"<<std::endl;
}

bool TriangulationLaser::isConnected() const {
    return serial_connected_ || modbus_connected_;
}

bool TriangulationLaser::riftek_requestId(std::vector<uint8_t>* response) {
    if (mode_ != LaserMode::Riftek || !serial_connected_) {
        std::cerr << "[Laser] Not connected in Riftek mode\n";
        return false;
    }

    auto req = buildRiftekRequest(riftek_address_, static_cast<uint8_t>(RiftekCommand::ID));
    boost::system::error_code ec;

    serial_port_->write_some(boost::asio::buffer(req), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }

    std::vector<uint8_t> raw_resp(14);
    size_t len = serial_port_->read_some(boost::asio::buffer(raw_resp), ec);
    if (ec) {
        std::cerr << "[Laser] Read error: " << ec.message() << std::endl;
        return false;
    }

    raw_resp.resize(len);

    if (response) {
        if (!parseRiftekResponse(raw_resp, *response)) {
            return false;
        }
    }

    return true;
}

bool TriangulationLaser::riftek_requestResult(uint16_t& rawValue) {
    if (mode_ != LaserMode::Riftek || !serial_connected_) {
        std::cerr << "[Laser] Not connected in Riftek mode\n";
        return false;
    }

    auto req = buildRiftekRequest(riftek_address_, static_cast<uint8_t>(RiftekCommand::REQUEST_RESULT));
    boost::system::error_code ec;

    serial_port_->write_some(boost::asio::buffer(req), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }

    std::vector<uint8_t> raw_resp(4);
    size_t len = serial_port_->read_some(boost::asio::buffer(raw_resp), ec);
    if (ec) {
        std::cerr << "[Laser] Read error: " << ec.message() << std::endl;
        return false;
    }

    std::vector<uint8_t> data;
    if (!parseRiftekResponse(raw_resp, data) || data.size() < 2) {
        std::cerr << "[Laser] Invalid response for result request"<<std::endl;
        return false;
    }

    rawValue = (static_cast<uint16_t>(data[1]) << 8) | data[0];
    return true;
}

bool TriangulationLaser::riftek_startStream() {
    if (mode_ != LaserMode::Riftek || !serial_connected_) {
        std::cerr << "[Laser] Not connected in Riftek mode"<<std::endl;
        return false;
    }

    auto req = buildRiftekRequest(riftek_address_, static_cast<uint8_t>(RiftekCommand::START_STREAM));
    boost::system::error_code ec;

    serial_port_->write_some(boost::asio::buffer(req), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }

    riftek_streaming_ = true;
    std::cout << "[Laser] Started stream"<<std::endl;
    return true;
}

bool TriangulationLaser::riftek_stopStream() {
    if (mode_ != LaserMode::Riftek || !serial_connected_) {
        std::cerr << "[Laser] Not connected in Riftek mode"<<std::endl;
        return false;
    }

    auto req = buildRiftekRequest(riftek_address_, static_cast<uint8_t>(RiftekCommand::STOP_STREAM));
    boost::system::error_code ec;

    serial_port_->write_some(boost::asio::buffer(req), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }

    riftek_streaming_ = false;
    std::cout << "[Laser] Stopped stream"<<std::endl;
    return true;
}

bool TriangulationLaser::riftek_writeParameter(uint8_t paramCode, uint8_t value) {
    if (mode_ != LaserMode::Riftek || !serial_connected_) {
        std::cerr << "[Laser] Not connected in Riftek mode"<<std::endl;
        return false;
    }

    std::vector<uint8_t> msg = {paramCode, value};
    auto req = buildRiftekRequest(riftek_address_, static_cast<uint8_t>(RiftekCommand::WRITE_PARAM), msg);
    boost::system::error_code ec;

    serial_port_->write_some(boost::asio::buffer(req), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }

    return true;
}

bool TriangulationLaser::riftek_readParameter(uint8_t paramCode, uint8_t& value) {
    if (mode_ != LaserMode::Riftek || !serial_connected_) {
        std::cerr << "[Laser] Not connected in Riftek mode"<<std::endl;
        return false;
    }

    std::vector<uint8_t> msg = {paramCode};
    auto req = buildRiftekRequest(riftek_address_, static_cast<uint8_t>(RiftekCommand::READ_PARAM), msg);
    boost::system::error_code ec;

    serial_port_->write_some(boost::asio::buffer(req), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }

    std::vector<uint8_t> raw_resp(4);
    size_t len = serial_port_->read_some(boost::asio::buffer(raw_resp), ec);
    if (ec) {
        std::cerr << "[Laser] Read error: " << ec.message() << std::endl;
        return false;
    }

    std::vector<uint8_t> data;
    if (!parseRiftekResponse(raw_resp, data) || data.size() < 1) {
        std::cerr << "[Laser] Invalid response for parameter read"<<std::endl;
        return false;
    }

    value = data[0];
    return true;
}

bool TriangulationLaser::riftek_flashAction(RiftekFlashAction action) {
    if (mode_ != LaserMode::Riftek || !serial_connected_) {
        std::cerr << "[Laser] Not connected in Riftek mode"<<std::endl;
        return false;
    }

    uint8_t msgByte = (action == RiftekFlashAction::Save) ? 0xAA : 0x69;
    std::vector<uint8_t> msg = {msgByte};
    auto req = buildRiftekRequest(riftek_address_, static_cast<uint8_t>(RiftekCommand::FLASH), msg);
    boost::system::error_code ec;

    serial_port_->write_some(boost::asio::buffer(req), ec);
    if (ec) {
        std::cerr << "[Laser] Write error: " << ec.message() << std::endl;
        return false;
    }

    std::vector<uint8_t> raw_resp(4);
    size_t len = serial_port_->read_some(boost::asio::buffer(raw_resp), ec);
    if (ec) {
        std::cerr << "[Laser] Read error: " << ec.message() << std::endl;
        return false;
    }

    std::vector<uint8_t> data;
    if (!parseRiftekResponse(raw_resp, data) || data.size() < 1) {
        std::cerr << "[Laser] Invalid response for flash action"<<std::endl;
        return false;
    }

    if (data[0] != msgByte) {
        std::cerr << "[Laser] Flash action failed: unexpected response"<<std::endl;
        return false;
    }

    std::cout << "[Laser] Flash action completed successfully"<<std::endl;
    return true;
}

bool TriangulationLaser::riftek_switchToModbus() {
    return riftek_writeParameter(
        static_cast<uint8_t>(RiftekParameter::PROTOCOL_MODE),
        0x02
    );
}

bool TriangulationLaser::modbus_setLaserEnabled(bool enable) {
    if (!modbus_connected_) {
        std::cerr << "[Laser] Not connected!" << std::endl;
        return false;
    }
    uint16_t state = enable ? 1 : 0;
    if (modbus_write_register(ctx_, 10, state) != 1) {
        handleError("modbus_write_register");
        return false;
    }
    std::cout << "[Laser] " << (enable ? "ON" : "OFF") << std::endl;
    return true;
}

bool TriangulationLaser::modbus_readRawMeasurement(uint16_t& rawValue) {
    // Nothing to do
    return true;
}