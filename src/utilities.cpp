#include "utilities.h"

#include <ctime>


std::string utilities::timestamp(){

    std::time_t now = std::time(nullptr);
    std::tm * ptm = std::localtime(&now);
    char buffer[100];
    std::strftime(buffer, 70, "%F_%H%M%S", ptm); 
    return std::string(buffer);

}

std::string utilities::create_logfile(){
    return "logs/drone_" + timestamp() + ".log";
}


std::string utilities::bitmap_to_str(uint32_t bitmap){

    std::string retval;
    for(int i =0; i < 32; i++){
        retval += std::to_string(0b1 & bitmap);
        bitmap>>=1;
    }
    return retval;
}

std::string utilities::bitmap_to_str(uint16_t bitmap){
    std::string retval;
    for(int i =0; i < 16; i++){
        retval += std::to_string(0b1 & bitmap);
        bitmap>>=1;
    }
    return retval;
}