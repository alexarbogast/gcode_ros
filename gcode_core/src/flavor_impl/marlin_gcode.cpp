#include <Eigen/Geometry>
#include "gcode_core/utils/command_utils.h"
#include "gcode_core/flavor_impl/marlin_gcode.h"


//////////
#include <iostream>

namespace gcode_core
{

// ss is a single line of gcode
void MarlinGcode::parse(std::stringstream& ss)
{
    std::string command_token;
    ss >> command_token;
    
    // treat extrusion and non-extrusion moves identically (for now)
    if (command_token == "G1" || command_token == "G0")
    {
        MarlinMoveCommand cmd;

        // initialize move command as previous
        const MoveCommand* prev = getLastMoveCommand(*this);
        if (prev)
            cmd.setWaypoint(prev->getWaypoint());
            
        ss >> cmd;
        container_.push_back(cmd);
    }  
}

} // namespace gcode_core
