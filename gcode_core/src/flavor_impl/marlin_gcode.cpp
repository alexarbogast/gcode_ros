#include "gcode_core/flavor_impl/marlin_gcode.h"
#include <iostream>

namespace gcode_core
{
namespace Marlin
{
BeadType ParseBeadType(const std::string& line)
{
    BeadType type = BeadType::None;

    if (line.find("WALL-INNER") != std::string::npos)
    {
        type = BeadType::WallInner;
    }
    else if (line.find("WALL-OUTER") != std::string::npos)
    {
        type = BeadType::WallOuter;
    }
    else if (line.find("SKIN") != std::string::npos)
    {
        type = BeadType::Skin;
    }
    else if (line.find("FILL") != std::string::npos)
    {
        type = BeadType::Fill;
    }

    return type;
}

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
    MoveCommand cmd;

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
            if (!toolpath.back().empty())
                if(!toolpath.back().back().empty())
                    toolpath.back().back().pop_back();

            std::shared_ptr<Bead> bead = std::make_shared<Bead>(ParseBeadType(command_token));
            
            // transfer previous travel move to this bead
            bead->push_back(std::make_shared<MoveCommand>(cmd));
            toolpath.back().push_back(bead);
        }
        else if (command_token == "G1" || command_token == "G0")                     // move command
        {
            MoveCommandType type = MoveCommandType::Travel;

            if (command_token == "G1")
                type = MoveCommandType::Extrusion;
            
            // initialized as previous command  
            MoveCommand new_cmd(cmd);
            ParseMoveCommand(ss, new_cmd);
            new_cmd.setCommandType(type);

            if (!new_cmd.getWaypoint().isApprox(cmd.getWaypoint()))
                if (!toolpath.empty())
                    if (!toolpath.back().empty())
                        toolpath.back().back().push_back(std::make_shared<MoveCommand>(new_cmd));

            cmd = new_cmd;
        }
    }
}

} // namespace Marlin 
} // namespace gcode_core