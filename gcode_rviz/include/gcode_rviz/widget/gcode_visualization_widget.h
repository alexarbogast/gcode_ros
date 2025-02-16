#ifndef GCODE_VISUALIZATION_WIDGET_H
#define GCODE_VISUALIZATION_WIDGET_H

#include <gcode_rviz/widget/layer_slider.h>
#include <gcode_rviz/widget/layer_range_visualization.h>
#include <gcode_core/core/toolpath.h>

#include <QSpinBox>
#include <QLineEdit>
#include <QSlider>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>

namespace gcode_rviz
{
class GcodeVisualizationWidget : public QWidget
{
  Q_OBJECT
public:
  GcodeVisualizationWidget(QWidget* parent = 0);
  virtual ~GcodeVisualizationWidget();

  void setToolpath(const gcode_core::ToolpathPtr& toolpath);
  void displayGcodeLayerRange();

protected:
  LayerRangeVisualizationPtr layer_range_vis_;
  LayerRangeWidget* layer_range_;
};

}  // namespace gcode_rviz

#endif  // GCODE_VISUALIZATION_WIDGET_H
