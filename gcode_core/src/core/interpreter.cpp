#include "gcode_core/core/interpreter.h"
#include <fstream>

namespace gcode_core
{
void GcodeInterpreter::parseGcode(const std::string& filepath,
                                  Toolpath& toolpath)
{
  std::ifstream filein(filepath, std::ios::in);
  move_command_ = MoveCommand();
  previous_extruder_ = 0.0;

  for (std::string line; std::getline(filein, line);)
  {
    std::stringstream ss(line);
    parseLine(ss, toolpath);
  }
}

void GcodeInterpreter::parseLine(std::stringstream& ss, Toolpath& toolpath)
{
  std::string command_token;
  ss >> command_token;

  if (command_token == "G1" || command_token == "G0")
  {

    MoveCommand old_cmd = move_command_; 
    parseMoveCommand(ss, move_command_);

    // only add move commands with non-zero travel
    double d = (old_cmd.translation() - move_command_.translation()).norm();
    if (d)
    {
      toolpath.push_back(std::make_shared<MoveCommand>(move_command_));
    }
  }
  else if (command_token == "G92")
  {
    parseMoveCommand(ss, move_command_);
  }
  else if (command_token[0] == 'T')
  {
    std::string tool = command_token.substr(1, command_token.size());
    move_command_.setTool(std::stoi(tool));
  }
}

void GcodeInterpreter::parseMoveCommand(std::stringstream& ss, MoveCommand& cmd)
{
  MoveCommand::TranslationPart trans = cmd.translation();
  cmd.setCommandType(MoveCommandType::Travel);
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
      case 'E': {
        double ext = std::stod(token.substr(1));
        if (ext - previous_extruder_ > 0.0)
        {
          cmd.setCommandType(MoveCommandType::Extrusion);
        }
        previous_extruder_ = ext;
        break;
      }
      default:
        break;
    }
  }
}

}  // namespace gcode_core
