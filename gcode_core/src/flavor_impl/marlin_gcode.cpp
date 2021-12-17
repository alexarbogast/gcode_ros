#include "gcode_core/flavor_impl/marlin_gcode.h"

namespace gcode_core
{
// ss is a single line of gcode
void MarlinGcode::parse(std::stringstream& ss)
{
    std::string command_token;
    ss >> command_token;
    
    if (command_token == "G1")
    {
        MarlinMoveCommand cmd;
        ss >> cmd;
        container_.push_back(cmd);
    }  
}

} // namespace gcode_core
