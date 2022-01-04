#include "gcode_rosutils/plotting.h"

namespace gcode_rosutils
{
GcodePlotter::GcodePlotter(const std::string& root_link, const std::string& topic_namespace)
    : root_link_(root_link), topic_namespace_(topic_namespace)
{
    ros::NodeHandle nh;
    gcode_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_namespace + "/display_gcode", 1, true);
    
}

} // namespace gcode_rosutils