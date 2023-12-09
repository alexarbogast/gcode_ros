#ifndef INTERPRETER_INTERFACE_H
#define INTERPRETER_INTERFACE_H

#include <string>
#include "gcode_core/core/gcode.h"

namespace gcode_core
{

class GcodeInterpreter
{
public:
  GcodeInterpreter() = default;
  virtual ~GcodeInterpreter() = default;

  virtual void parseGcode(const std::string& filepath, Toolpath& toolpath);

protected:
  virtual void parseLine(std::stringstream& ss, Toolpath& toolpath);
  MoveCommand move_command_;
  double previous_extruder_;

private:
  virtual void parseMoveCommand(std::stringstream& ss, MoveCommand& cmd);
};

}  // namespace gcode_core

#endif  // INTERPRETER_INTERFACE_H
