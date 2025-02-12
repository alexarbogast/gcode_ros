#ifndef GCODE_BASE_H
#define GCODE_BASE_H

#include "gcode_core/core/toolpath.h"
#include "gcode_core/core/macros.h"

namespace gcode_core
{
GCODE_CORE_CLASS_FORWARD(GcodeBase);

class GcodeBase
{
public:
  GcodeBase() = default;
  virtual ~GcodeBase() = default;

  Toolpath& toolpath() { return *toolpath_; }
  const Toolpath& toolpath() const { return *toolpath_; }

private:
  std::unique_ptr<Toolpath> toolpath_ = std::make_unique<Toolpath>();
};

}  // namespace gcode_core

#endif  // GCODE_BASE_H
