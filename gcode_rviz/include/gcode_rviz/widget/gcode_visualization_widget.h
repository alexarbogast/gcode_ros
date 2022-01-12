#ifndef GCODE_VISUALIZATION_WIDGET_H
#define GCODE_VISUALIZATION_WIDGET_H

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <QSpinBox>
#include <QSlider>
#include <QPushButton>

#include "gcode_rviz/widget/color_list_editor.h"
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
    void DisplayGcodeLayerRange();

protected Q_SLOTS:
    void set_min_layer(int value);
    void set_max_layer(int value);
    void pick_color_click();

protected:
    GcodeBaseConstPtr gcode_ = nullptr;
    rviz_visual_tools::RvizVisualToolsPtr rvt_;

    QSlider* min_layer_slider_;
    QSlider* max_layer_slider_;
    QSpinBox* min_layer_spinbox_;
    QSpinBox* max_layer_spinbox_;
    
    QDoubleSpinBox* line_width_;
    QComboBox* display_style_;
    QComboBox* color_method_;
    QPushButton* pick_color_button_; 
    QColor layer_color_;
};

} // namespace gcode_rviz

#endif // GCODE_VISUALIZATION_WIDGET_H