#ifndef MARLIN_MOVE_COMMAND_H
#define MARLIN_MOVE_COMMAND_H

#include <Eigen/Geometry>
#include "gcode_core/move_command.h"

namespace gcode_core
{
class MarlinMoveCommand : public MoveCommand
{
public:
    MarlinMoveCommand() = default;

    virtual void parse(std::stringstream& args) override;
    virtual void print() const override;
};

} //namespace gcode_core

#endif // MARLIN_MOVE_COMMAND_H