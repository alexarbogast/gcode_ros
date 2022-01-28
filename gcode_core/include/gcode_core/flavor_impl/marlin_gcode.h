#ifndef MARLIN_GCODE_READER_H
#define MARLIN_GCODE_READER_H

#include <string>
#include <fstream>
#include <sstream>
#include <memory>

#include "gcode_core/core/gcode.h"
#include "gcode_core/core/toolpath.h"
#include "gcode_core/core/move_command.h"

namespace gcode_core
{
namespace Marlin
{
void ParseGcode(const std::string& filepath, GcodeBase& gcode_object);
void ParseMoveCommand(std::stringstream& ss, MoveCommand& cmd_object);
BeadType ParseBeadType(const std::string& line);

} // namespace marlin
} // namespace gcode_core

#endif // MARLIN_GCODE_READER_H
