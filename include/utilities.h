#pragma once

#include <string>

namespace utilities{
    std::string timestamp();
    std::string create_logfile();
    std::string bitmap_to_str(uint32_t bitmap);
    std::string bitmap_to_str(uint16_t bitmap);

}