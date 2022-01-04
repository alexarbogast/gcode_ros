#ifndef GCODE_VISUALIZATION_WIDGET_H
#define GCODE_VISUALIZATION_WIDGET_H

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#endif

// forward declarations
namespace rviz
{
    class Property;
} // namespace rviz

namespace gcode_rviz
{
class GcodeVisualizationWidget : public QObject
{
public:
    GcodeVisualizationWidget(rviz::Property* widget, rviz::Display* display);
    virtual ~GcodeVisualizationWidget();

    void setDisplayTrajectory(const visualization_msgs::MarkerArray::ConstPtr& msg);

private:
    rviz::Property* widget_;
    rviz::Display* display_;
};

} // namespace gcode_rviz

#endif // GCODE_VISUALIZATION_WIDGET_H