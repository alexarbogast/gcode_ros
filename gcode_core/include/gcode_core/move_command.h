#ifndef MOVE_COMMAND_H
#define MOVE_COMMAND_H

#include "gcode_core/core/command.h"

namespace gcode_core
{
class MoveCommand
{
public:
    MoveCommand() = default;
    virtual void parse(std::stringstream& args) = 0;
    virtual void print() const = 0;

    friend std::stringstream& operator>>(std::stringstream& ss, MoveCommand& cmd)
    {
        cmd.parse(ss);
        return ss;
    }
protected:
    float x_ = 0, y_ = 0, z_ = 0;
};

} //namespace gcode_core

#endif // MOVE_COMMAND_H