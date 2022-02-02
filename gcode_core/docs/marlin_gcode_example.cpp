#include <iostream>
#include <ros/package.h>

#include "gcode_core/flavor_impl/marlin_gcode.h"

using namespace gcode_core;

int main()
{
    // find example marlin gcode filepath 
    std::string filepath = ros::package::getPath("gcode_core") + "/resources/simple_cube_marlin.gcode";

    // Instantiate gcode object and parse file 
    GcodeBase gcode;
    Marlin::ParseGcode(filepath, gcode);
    
    // Retrieve the toolpath from the gcode
    Toolpath& toolpath = gcode.toolpath();
    
    // print toolpath to console
    std::cout << toolpath << std::endl;
    return 0;
}