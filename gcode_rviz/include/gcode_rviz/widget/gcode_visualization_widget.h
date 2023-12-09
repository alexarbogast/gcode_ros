#ifndef GCODE_VISUALIZATION_WIDGET_H
#define GCODE_VISUALIZATION_WIDGET_H

#include <unordered_map>
#include <QSpinBox>
#include <QLineEdit>
#include <QSlider>
#include <QPushButton>
#include <QCheckBox>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include "gcode_core/core/gcode.h"
#include "gcode_rviz/widget/color_list_editor.h"
#include "gcode_rviz/widget/layer_slider.h"
#include "gcode_rviz/widget/layer_range_visualization.h"

using namespace gcode_core;

namespace gcode_rviz
{
class GcodeVisualizationWidget : public QWidget
{
  Q_OBJECT
public:
  GcodeVisualizationWidget(QWidget* parent = 0);
  virtual ~GcodeVisualizationWidget();

  void SetToolpath(std::shared_ptr<Toolpath> toolpath);
  void DisplayGcodeLayerRange();

protected Q_SLOTS:
  void set_gcode_frame();
  void set_line_width(double value);
  void set_hide_travel(bool value);
  void set_color_method();
  void pick_color_click();
  void set_display_style(int value);

protected:
  LayerRangeVisualizationPtr layer_range_vis_;

  QLineEdit* gcode_frame_;
  LayerSliderWidget* layer_slider_;

  QDoubleSpinBox* line_width_;
  QComboBox* display_style_;
  QComboBox* color_method_;
  QPushButton* pick_color_button_;
  QColor layer_color_;
  QCheckBox* hide_travel_;
};

}  // namespace gcode_rviz

#endif  // GCODE_VISUALIZATION_WIDGET_H
