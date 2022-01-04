#ifndef GCODE_CORE_COMMAND_TYPE
#define GCODE_CORE_COMMAND_TYPE

#include "gcode_core/core/command.h"
#include "gcode_core/move_command.h"
#include "gcode_core/core/gcode_base.h"

namespace gcode_core
{
class Command;
class MoveCommand;
class GcodeBase;

inline MoveCommand* getLastMoveCommand(GcodeBase& gcode)
{
    auto cmd_iter = std::find_if(gcode.rbegin(), gcode.rend(),
            [](const Command& cmd) { return cmd.GetCommandType() == MoveCommand::GetStaticType(); });

    if (cmd_iter != std::rend(gcode))
        return &cmd_iter->as<MoveCommand>();

    return nullptr;
}

inline const MoveCommand* getLastMoveCommand(const GcodeBase& gcode)
{
    const auto cmd_iter = std::find_if(gcode.rbegin(), gcode.rend(),
            [](const Command& cmd) { return cmd.GetCommandType() == MoveCommand::GetStaticType(); });

    if (cmd_iter != std::rend(gcode))
        return &cmd_iter->as<MoveCommand>();

    return nullptr;
}

} // namespace gcode_core

#endif // GCODE_CORE_COMMAND_TYPE