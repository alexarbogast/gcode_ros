#include <iostream>
#include "gcode_core/flavor_impl/marlin_move_command.h"

namespace gcode_core
{
void MarlinMoveCommand::parse(std::stringstream& args)
{
    TranslationPart trans = waypoint_.translation();
    std::string token;

    while (args >> token)
    {
        char c = token[0];
        switch (toupper(c))
        {
            case 'X':
                trans[0] = std::stod(token.substr(1));
                break;
            case 'Y':
                trans[1] = std::stod(token.substr(1));
                break;
            case 'Z':
                trans[2] = std::stod(token.substr(1));
                break;
            default:
                break;
        }
    }
}

void MarlinMoveCommand::print() const
{
    std::stringstream ss;
    ConstTranslationPart trans = waypoint_.translation();

    ss << "X" << std::to_string(trans[0]) << " "
       << "Y" << std::to_string(trans[1]) << " "
       << "Z" << std::to_string(trans[2]) << " ";
    
    std::cout << ss.str() << std::endl;
}

} // namespace gcode_core

