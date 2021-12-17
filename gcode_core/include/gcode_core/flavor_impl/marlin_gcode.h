#ifndef MARLIN_GCODE_H
#define MARLIN_GCODE_H

#include "gcode_core/core/gcode_base.h"
#include "gcode_core/flavor_impl/marlin_move_command.h"

namespace gcode_core
{
class MarlinGcode : public GcodeBase
{  
public:
    virtual void parse(std::stringstream& ss) override;
};

} // namespace gcode_core

#endif // MARLIN_GCODE_H
