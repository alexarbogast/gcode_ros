#include <gcode_core/core/conversions.h>

#include <gcode_msgs/Move.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <unordered_map>
#include "gcode_core/core/move_command.h"

namespace gcode_core
{

static std::unordered_map<MoveCommandType, uint8_t> UnderlyingType = {
  { MoveCommandType::None, gcode_msgs::Move::NONE },
  { MoveCommandType::Travel, gcode_msgs::Move::TRAVEL },
  { MoveCommandType::Extrusion, gcode_msgs::Move::EXTRUSION }
};

void moveToMsg(const MoveCommand& cmd, gcode_msgs::Move& msg)
{
  tf2::convert(cmd.getWaypoint(), msg.pose);
  msg.type = UnderlyingType[cmd.getCommandType()];
  msg.tool = cmd.getTool();
}

void toolpathToMsg(const Toolpath& toolpath, gcode_msgs::Toolpath& msg)
{
  msg.moves.reserve(toolpath.size());
  for (const auto& cmd : toolpath)
  {
    gcode_msgs::Move move;
    moveToMsg(*cmd, move);
    msg.moves.emplace_back(std::move(move));
  }
  msg.action = gcode_msgs::Toolpath::ADD;
}

}  // namespace gcode_core
