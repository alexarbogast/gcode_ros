#include <fstream>
#include <iostream>

#include "gcode_core/gcode_reader.h"
#include <vector>
#include <algorithm>
#include "gcode_core/flavor_impl/marlin_gcode.h"

using namespace gcode_core;

int main()
{
    std::string filepath = "/home/alex/Desktop/cylindermod.gcode";

    MarlinGcode gcode;
    GcodeReader::ParseGcode(filepath, gcode);

    gcode.print();
    return 0;
}