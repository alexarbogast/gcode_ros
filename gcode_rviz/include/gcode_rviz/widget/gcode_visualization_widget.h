#ifndef GCODE_VISUALIZATION_WIDGET_H
#define GCODE_VISUALIZATION_WIDGET_H

#include <unordered_map>
#include <QSpinBox>
#include <QLineEdit>
#include <QSlider>
#include <QPushButton>
#include <QCheckBox>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include "gcode_rviz/widget/color_list_editor.h"
#include "gcode_core/core/gcode.h"

using namespace gcode_core;

namespace gcode_rviz
{
std_msgs::ColorRGBA create_color_rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

const std_msgs::ColorRGBA WALL_OUTER_COLOR = create_color_rgba(223, 91, 30, 255);
const std_msgs::ColorRGBA WALL_INNER_COLOR = create_color_rgba(58, 183, 149, 255);
const std_msgs::ColorRGBA SKIN_COLOR = create_color_rgba(139, 133, 193, 255);  
const std_msgs::ColorRGBA FILL_COLOR = create_color_rgba(255, 207, 86, 255);

class GcodeVisualizationWidget : public QWidget
{
Q_OBJECT
public:
    GcodeVisualizationWidget(QWidget* parent = 0);
    virtual ~GcodeVisualizationWidget();

    void SetGcode(GcodeBaseConstPtr gcode);
    void DisplayGcodeLayerRange();

protected Q_SLOTS:
    void set_gcode_frame();
    void set_min_layer(int value);
    void set_max_layer(int value);
    void pick_color_click();

protected:
    GcodeBaseConstPtr gcode_ = nullptr;
    rviz_visual_tools::RvizVisualToolsPtr rvt_;

    QLineEdit* gcode_frame_;
    QSlider* min_layer_slider_;
    QSlider* max_layer_slider_;
    QSpinBox* min_layer_spinbox_;
    QSpinBox* max_layer_spinbox_;
    
    QDoubleSpinBox* line_width_;
    QComboBox* display_style_;
    QComboBox* color_method_;
    QPushButton* pick_color_button_; 
    QColor layer_color_;
    QCheckBox* hide_travel_;
};

} // namespace gcode_rviz

#endif // GCODE_VISUALIZATION_WIDGET_H