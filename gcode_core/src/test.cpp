#include <fstream>
#include <iostream>

#include <vector>
#include <algorithm>
#include "gcode_core/flavor_impl/marlin_gcode.h"

// TEMP
#include "gcode_core/core/toolpath.h"

using namespace gcode_core;

int main()
{
    std::string filepath = "/home/alex/Documents/parts/cube_bottom.gcode";

    GcodeBase gcode;
    Marlin::ParseGcode(filepath, gcode);
    
    Toolpath& toolpath = gcode.toolpath();
    
    std::cout << toolpath << std::endl;
    return 0;
}