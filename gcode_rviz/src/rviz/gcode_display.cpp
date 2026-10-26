#include <gcode_rviz/rviz/gcode_display.h>

namespace gcode_rviz
{
GcodeDisplay::GcodeDisplay() : rviz_common::Display() {}

GcodeDisplay::~GcodeDisplay() {}

void GcodeDisplay::onInitialize() {}

}  // namespace gcode_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gcode_rviz::GcodeDisplay, rviz_common::Display)
