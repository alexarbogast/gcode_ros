#ifndef GCODE_CONVERSIONS_H
#define GCODE_CONVERSIONS_H

#include <gcode_core/core/move_command.h>
#include <gcode_core/core/toolpath.h>

#include <gcode_msgs/Move.h>
#include <gcode_msgs/Toolpath.h>

namespace gcode_core
{
void moveToMsg(const MoveCommand& cmd, gcode_msgs::Move& msg);
void toolpathToMsg(const Toolpath& toolpath, gcode_msgs::Toolpath& msg);

}  // namespace gcode_core

#endif  // GCODE_CONVERSIONS_H
