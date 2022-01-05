#ifndef GCODE_VISUALIZATION_WIDGET_H
#define GCODE_VISUALIZATION_WIDGET_H

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <QWidget>
#include <QPushButton>

#include "gcode_core/core/gcode_base.h"

using namespace gcode_core;

namespace gcode_rviz
{
class GcodeVisualizationWidget : public QWidget
{
Q_OBJECT
public:
    GcodeVisualizationWidget(QWidget* parent = 0);
    virtual ~GcodeVisualizationWidget();

    void DisplayGcode(GcodeBaseConstPtr gcode);
protected:
    rviz_visual_tools::RvizVisualToolsPtr rvt_;
};

} // namespace gcode_rviz

#endif // GCODE_VISUALIZATION_WIDGET_H