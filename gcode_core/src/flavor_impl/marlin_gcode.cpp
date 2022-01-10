#include "gcode_core/flavor_impl/marlin_gcode.h"

namespace gcode_core
{
namespace Marlin
{
void ParseMoveCommand(std::stringstream& ss, MoveCommand& cmd_object)
{
    MoveCommand::TranslationPart trans = cmd_object.translation();
    std::string token;

    while (ss >> token)
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

void ParseGcode(const std::string& filepath, GcodeBase& gcode_object)
{
    Toolpath& toolpath = gcode_object.toolpath();

    std::ifstream filein(filepath, std::ios::in);
    for (std::string line; std::getline(filein, line);)
    {
        std::stringstream ss(line);
        std::string command_token;
        ss >> command_token;

        if (command_token.find(";LAYER:") != std::string::npos)                      // new layer
        {
            toolpath.push_back(std::make_shared<Layer>());           
        }
        else if (command_token.find(";TYPE:") != std::string::npos)                  // new bead
        {
            toolpath.back().push_back(std::make_shared<Bead>());
        }
        else if (command_token == "G1" || command_token == "G0")                     // move command
        {
            if (!toolpath.empty())
            {
                if (!toolpath.back().empty())
                {
                    MoveCommand cmd;
                    ParseMoveCommand(ss, cmd);

                    toolpath.back().back().push_back(std::make_shared<MoveCommand>(cmd));
                }
            }
        }
    }
}

} // namespace Marlin 
} // namespace gcode_core