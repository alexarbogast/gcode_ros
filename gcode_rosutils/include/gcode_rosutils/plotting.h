#ifndef GCODE_ROSUTILS_PLOTTING_H
#define GCODE_ROSUTILS_PLOTTING_H

#include <ros/ros.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

namespace gcode_rosutils
{
class GcodePlotter
{
public:
    GcodePlotter(const std::string& root_link = "world", const std::string& topic_namespace = "gcode_ros");

private:
    std::string root_link_;
    std::string topic_namespace_;

    ros::Publisher gcode_pub_;
};

} // namespace gcode_rosutils

#endif // GCODE_ROSUTILS_PLOTTING_H