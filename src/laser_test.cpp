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
    std::cout<<"\nPress ENTER to continue...";
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

    std::cout<<'='*5+"/\tlaser tester\t/"+'='*5<<std::endl;
    std::cout<<"Device:\t\t"<<default_config.deviceName<<std::endl;
    std::cout<<"Port:\t\t"<<default_config.port<<std::endl;
    std::cout<<"Baudrate:\t\t"<<default_config.baudrate<<std::endl;
    std::cout<<"Parity:\t\t"<<default_config.parity<<std::endl;
    std::cout<<"Data bits:\t\t"<<default_config.dataBits<<std::endl;
    std::cout<<"Stop bits:\t\t"<<default_config.stopBits<<std::endl;
    std::cout<<"Slave ID:\t\t"<<default_config.slaveId<<std::endl;
    std::cout<<"Sensor range:\t\t"<<default_config.sensorRangeMm<<" mm"<<std::endl;
    std::cout<<"Update every:\t\t"<<default_config.updateIntervalMs<<" ms"<<std::endl;
    waitForEnter();

    // to be continued

    return 0;
}