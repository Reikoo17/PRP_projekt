#include <iostream>

#include "../include/Robot.h"
#include "../include/UDPSocket.h"


int main(int argc, char *argv[]) {

    Robot LoL(1,1,argc,argv);

   //std::cout << UDPSocket::string_to_nmea_message("RESET,0") << std::endl;

    std::cout << "Hello, World!" << std::endl;
    return 0;
}
