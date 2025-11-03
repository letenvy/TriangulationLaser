#include "TriangulationLaser.hpp"
#include "LaserConfig.hpp"
#include "ConsoleInput.hpp"

#include <iostream>
#include <thread>
#include <chrono>

LaserConfig default_config ={
    .deviceName="RIFTEK RF 603 125/500-232-U",
#ifdef _WIN32
    .port = "COM3",
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

    std::cout<<"\n\nConnecting to laser module . . ."<<std::endl;

    TriangulationLaser laser(
        default_config.port,
        default_config.baudrate,
        default_config.parity,
        default_config.dataBits,
        default_config.stopBits,
        default_config.slaveId,
        default_config.sensorRangeMm
    );

    if(!laser.connect()){
        std::cerr<<"Failed to connect to the laser!"<<std::endl;
        return 1;
    }

    std::cout<<"Connected successfully!"<<std::endl;

    waitForEnter();

    std::cout<<"Switching to MODBUS. . ."<<std::endl;
    laser.switchToModbus();
    std::cout<<"Switched successfully!"<<std::endl;
    waitForEnter();
    clearConsole();

    // to be continued

    return 0;
}