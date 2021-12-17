#include <iostream>
#include "gcode_core/flavor_impl/marlin_move_command.h"

namespace gcode_core
{
MarlinMoveCommand::MarlinMoveCommand() { }

void MarlinMoveCommand::parse(std::stringstream& args)
{
    std::string token;
    while (args >> token)
    {
        char c = token[0];
        switch (toupper(c))
        {
            case 'X':
                x_ = std::stof(token.substr(1));
                break;
            case 'Y':
                y_ = std::stof(token.substr(1));
                break;
            case 'Z':
                z_ = std::stof(token.substr(1));
                break;
            default:
                break;
        }
    }
}

void MarlinMoveCommand::print() const
{
    std::stringstream ss;
    ss << "X" << std::to_string(x_) << " "
       << "Y" << std::to_string(y_) << " "
       << "Z" << std::to_string(z_) << " ";
    
    std::cout << ss.str() << std::endl;
}

} // namespace gcode_core

