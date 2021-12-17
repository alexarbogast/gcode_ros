#ifndef GCODE_READER_H
#define GCODE_READER_H

#include <string>
#include <fstream>
#include <sstream>

namespace gcode_core
{
class GcodeReader
{
public:    
    template <class T>
    static void ParseGcode(const std::string& filepath, T& gcode_object)
    {
        std::ifstream filein(filepath, std::ios::in);
        for (std::string line; std::getline(filein, line);)
        {
            std::stringstream ss(line);
            ss >> gcode_object;
        }
    }
};

} // namespace gcode_core

#endif // GCODE_READER_H