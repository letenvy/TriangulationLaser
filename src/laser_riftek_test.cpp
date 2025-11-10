#include "TriangulationLaser.hpp"
#include "LaserConfig.hpp"
#include "ConsoleInput.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <vector>
#include <iterator>
#include <iomanip>
#include <limits>
#include <fstream>
#include <algorithm>

#include <nlohmann/json.hpp>

LaserConfig default_config ={
    .deviceName="RIFTEK RF 603 125/500-232-U",
#ifdef _WIN32
    .port = "COM5",
#else
    .port ="/dev/ttyUSB0", // didn't check port for Linux yet 
#endif
    .baudrate = 9600,
    .parity = 'E',
    .dataBits = 8,
    .stopBits = 1,
    .slaveId = 1,
    .sensorRangeMm = 500.0,
    .updateIntervalMs = 100
};

void clearConsole(){
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

void waitForEnter(){
    std::cout<<"\nPress ENTER to continue . . .";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
}

void runLiveMode(TriangulationLaser& laser, int intervalMs);
void runLiveLoggingMode(TriangulationLaser& laser, int intervalMs,const std::string& mode, double targetValue);

int main(){
    std::cout<<"Detected OS: "
#ifdef _WIN32
    <<"Windows"
#else
    <<"Linux"
#endif
    <<std::endl;

    std::cout<<"==========/\tlaser tester\t/=========="<<std::endl;
    std::cout<<"Device:\t\t"<<default_config.deviceName<<std::endl;
    std::cout<<"Port:\t\t"<<default_config.port<<std::endl;
    std::cout<<"Baudrate:\t"<<default_config.baudrate<<std::endl;
    std::cout<<"Parity:\t\t"<<default_config.parity<<std::endl;
    std::cout<<"Data bits:\t"<<default_config.dataBits<<std::endl;
    std::cout<<"Stop bits:\t"<<default_config.stopBits<<std::endl;
    std::cout<<"Slave ID:\t"<<default_config.slaveId<<std::endl;
    std::cout<<"Sensor range:\t"<<default_config.sensorRangeMm<<" mm"<<std::endl;
    std::cout<<"Update every:\t"<<default_config.updateIntervalMs<<" ms"<<std::endl;
    waitForEnter();
    clearConsole();

    TriangulationLaser laser(
        default_config.port,
        default_config.baudrate,
        default_config.parity,
        default_config.dataBits,
        default_config.stopBits,
        default_config.slaveId,
        default_config.sensorRangeMm
    );

    std::cout<<"Connecting to device in RIFTEK mode . . ."<<std::endl;

    boost::asio::io_context io;
    boost::asio::serial_port serial(io);

    try{
        laser.setupSerialPort(serial);
        std::cout<<"Connected to "<<default_config.port<<std::endl;
    }catch(const std::exception&e){
        std::cerr<<"Failed to connect: "<<e.what()<<std::endl;
        return 1;
    }
    
    std::cout<<"Disabling laser (00h=0) . . ."<<std::endl;
    std::vector<uint8_t> disableCmd = {0x01U, 0x03U, 0x00U, 0x00U};
    boost::asio::write(serial, boost::asio::buffer(disableCmd));
    waitForEnter();
    clearConsole();

    std::cout<<"Enabling laser (00h=1) . . ."<<std::endl;
    std::vector<uint8_t> disableCmd = {0x01U, 0x03U, 0x00U, 0x01U};
    boost::asio::write(serial, boost::asio::buffer(disableCmd));
    waitForEnter();
    clearConsole();

    std::cout<<"Requesting measurement (06h) . . ."<<std::endl;
    std::vector<uint8_t> readResultCmd={0x01U,0x03U,0x06U,0x00U};
    boost::asio::write(serial,boost::asio::buffer(readResultCmd));

    std::vector<uint8_t> rawValue(2);
    try{
        boost::asio::read(serial,boost::asio::buffer(rawValue),boost::asio::transfer_exactly(2));
    }catch(const std::exception& e){
        std::cerr<<"Failed to read measurement: "<<e.what()<<std::endl;
        serial.close();
        return 1;
    }

    uint16_t raw=(rawValue[0]<<8|rawValue[1]);
    double mm=static_cast<double>(raw)*default_config.sensorRangeMm/16384.0;
    std::cout<<"Raw value: "<<raw<<std::endl;
    std::cout<<"Distance: "<<std::fixed<<std::setprecision(3)<<mm<<" mm"<<std::endl;

    std::cout<<"Disabling laser and closing connection . . ."<<std::endl;
    serial.close();

    std::cout<<"Goodbye";
    return 0;
}