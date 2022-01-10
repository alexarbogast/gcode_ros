#ifndef GCODE_VISUALIZATION_WIDGET_H
#define GCODE_VISUALIZATION_WIDGET_H

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <QSpinBox>
#include <QSlider>
#include <QPushButton>

#include "gcode_core/core/gcode.h"

using namespace gcode_core;

namespace gcode_rviz
{
class GcodeVisualizationWidget : public QWidget
{
Q_OBJECT
public:
    GcodeVisualizationWidget(QWidget* parent = 0);
    virtual ~GcodeVisualizationWidget();

    void SetGcode(GcodeBaseConstPtr gcode);

    void DisplayGcode();
    void DisplayGcodeLayerRange(unsigned int lower, unsigned int upper);

protected Q_SLOTS:
    void set_min_layer(int value);
    void set_max_layer(int value);

protected:
    GcodeBaseConstPtr gcode_;
    rviz_visual_tools::RvizVisualToolsPtr rvt_;

    QSlider* min_layer_slider_;
    QSlider* max_layer_slider_;
    QSpinBox* min_layer_spinbox_;
    QSpinBox* max_layer_spinbox_;
};

} // namespace gcode_rviz

#endif // GCODE_VISUALIZATION_WIDGET_H