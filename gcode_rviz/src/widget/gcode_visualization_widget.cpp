#include <gcode_msgs/Toolpath.h>
#include <gcode_rviz/widget/gcode_visualization_widget.h>
#include <gcode_core/core/conversions.h>

#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QColorDialog>

namespace gcode_rviz
{

GcodeVisualizationWidget::GcodeVisualizationWidget(QWidget* parent)
  : QWidget(parent)
{
  layer_range_vis_ =
      std::make_shared<LayerRangeVisualization>("/visualization_gcode");

  QVBoxLayout* main_layout = new QVBoxLayout();
  setLayout(main_layout);

  QGroupBox* group_box = new QGroupBox(tr("Gcode Visual"));
  QVBoxLayout* group_box_layout = new QVBoxLayout();
  group_box->setLayout(group_box_layout);

  // layer range
  layer_range_ = new LayerRangeWidget(this);

  group_box_layout->addWidget(layer_range_);
  group_box_layout->setAlignment(Qt::AlignTop);

  main_layout->addWidget(group_box);

  connect(layer_range_, &LayerRangeWidget::editingFinished, this,
          [this]() { displayGcodeLayerRange(); });
}

GcodeVisualizationWidget::~GcodeVisualizationWidget() {}

void GcodeVisualizationWidget::setToolpath(
    const gcode_core::ToolpathPtr& toolpath)
{
  std::vector<gcode_msgs::Toolpath::Ptr> layers_msg;
  std::vector<gcode_core::ToolpathPtr> layers = toolpath->layers();
  for (std::size_t i = 0; i < layers.size(); ++i)
  {
    gcode_msgs::Toolpath::Ptr msg = boost::make_shared<gcode_msgs::Toolpath>();
    msg->id = i;
    msg->header.frame_id  = "world";
    layers_msg.emplace_back(std::move(msg));
    toolpathToMsg(*layers[i], *layers_msg.back());
  }
  layer_range_vis_->setLayers(layers_msg);
  int n_layers = layer_range_vis_->nLayers() - 1;

  layer_range_->setRangeBounds(0, n_layers);
  layer_range_->setLowerValue(0);
  layer_range_->setUpperValue(n_layers);

  displayGcodeLayerRange();
}

void GcodeVisualizationWidget::displayGcodeLayerRange()
{
  int lower = layer_range_->getLowerValue();
  int upper = layer_range_->getUpperValue();

  layer_range_vis_->update(lower, upper);
}

}  // namespace gcode_rviz
