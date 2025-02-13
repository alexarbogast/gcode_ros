#ifndef GCODE_DISPLAY_H
#define GCODE_DISPLAY_H

#include <rviz/display.h>

namespace gcode_rviz
{
class GcodeDisplay : public rviz::Display
{
  Q_OBJECT
public:
  GcodeDisplay();
  ~GcodeDisplay() override;

  void onInitialize() override;
};

}  // namespace gcode_rviz

#endif  // GCODE_DISPLAY_H
