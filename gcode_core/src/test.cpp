#include <fstream>
#include <iostream>

#include "gcode_core/gcode_reader.h"
#include "gcode_core/flavor_impl/marlin_gcode.h"

using namespace gcode_core;

int main()
{
    std::string filepath = "/home/alex/Desktop/cylinder.gcode";

    MarlinGcode gcode;
    GcodeReader::ParseGcode(filepath, gcode);

    //gcode.print();

    return 0;
}