#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QColorDialog>

#include "gcode_rviz/widget/gcode_visualization_widget.h"

namespace gcode_rviz
{

GcodeVisualizationWidget::GcodeVisualizationWidget(QWidget* parent)
  : QWidget(parent)
{
  layer_range_vis_ =
      std::make_shared<LayerRangeVisualization>("world", "/rviz_visual_tools");
  layer_range_vis_->setLineWidth(DEFAULT_LINE_WIDTH / 1000);

  QVBoxLayout* main_layout = new QVBoxLayout();
  setLayout(main_layout);

  QGroupBox* group_box = new QGroupBox(tr("Gcode Visual"));
  QVBoxLayout* group_box_layout = new QVBoxLayout();
  group_box->setLayout(group_box_layout);

  // frame selection
  QHBoxLayout* frame_layout = new QHBoxLayout();
  gcode_frame_ = new QLineEdit(tr("world"));
  frame_layout->addWidget(new QLabel(tr("frame: ")));
  frame_layout->addWidget(gcode_frame_);
  layer_slider_ = new LayerSliderWidget();

  // line width layout
  QHBoxLayout* line_width_layout = new QHBoxLayout();
  line_width_ = new QDoubleSpinBox();
  line_width_->setMinimum(0.001);
  line_width_->setDecimals(2);
  line_width_->setSingleStep(0.5);
  line_width_->setValue(DEFAULT_LINE_WIDTH);
  line_width_layout->addWidget(new QLabel(tr("line width (mm): ")));
  line_width_layout->addWidget(line_width_);

  // display style
  display_style_ = new QComboBox();
  display_style_->addItems({ "lines", "cylinders" });

  QHBoxLayout* display_style_layout = new QHBoxLayout();
  display_style_layout->addWidget(new QLabel(tr("display style: ")));
  display_style_layout->addWidget(display_style_);

  // color method
  layer_color_ = QColor(221, 64, 58);
  color_method_ = new QComboBox();
  color_method_->addItems({ "by tool", "random by layer", "uniform layers" });
  pick_color_button_ = new QPushButton(tr("..."));
  pick_color_button_->setToolTip(tr("pick color"));

  QHBoxLayout* color_method_layout = new QHBoxLayout();
  color_method_layout->addWidget(new QLabel(tr("color method: ")));
  color_method_layout->addWidget(color_method_);
  color_method_layout->addWidget(pick_color_button_);

  // hide travel
  hide_travel_ = new QCheckBox();
  hide_travel_->setChecked(true);

  QHBoxLayout* hide_travel_layout = new QHBoxLayout();
  hide_travel_layout->addWidget(new QLabel(tr("hide travel: ")));
  hide_travel_layout->addWidget(hide_travel_, 0, Qt::AlignRight);

  group_box_layout->addLayout(frame_layout);
  group_box_layout->addWidget(layer_slider_);
  group_box_layout->addWidget(new QLabel(tr("Apperance: ")));
  group_box_layout->addLayout(line_width_layout);
  group_box_layout->addLayout(display_style_layout);
  group_box_layout->addLayout(color_method_layout);
  group_box_layout->addLayout(hide_travel_layout);
  group_box_layout->setAlignment(Qt::AlignTop);

  main_layout->addWidget(group_box);

  connect(gcode_frame_, &QLineEdit::textChanged, this,
          &GcodeVisualizationWidget::set_gcode_frame);
  connect(layer_slider_, &LayerSliderWidget::valueChanged, this,
          &GcodeVisualizationWidget::DisplayGcodeLayerRange);
  connect(line_width_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          &GcodeVisualizationWidget::set_line_width);
  connect(display_style_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &GcodeVisualizationWidget::set_display_style);
  connect(color_method_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &GcodeVisualizationWidget::set_color_method);
  connect(pick_color_button_, &QPushButton::clicked, this,
          &GcodeVisualizationWidget::pick_color_click);
  connect(hide_travel_, &QCheckBox::stateChanged, this,
          &GcodeVisualizationWidget::set_hide_travel);
}

GcodeVisualizationWidget::~GcodeVisualizationWidget()
{
}

// slots
void GcodeVisualizationWidget::set_gcode_frame()
{
  layer_range_vis_->setBaseFrame(gcode_frame_->text().toStdString());
  DisplayGcodeLayerRange();
}

void GcodeVisualizationWidget::set_line_width(double value)
{
  layer_range_vis_->setLineWidth(value / 1000);
  DisplayGcodeLayerRange();
}

void GcodeVisualizationWidget::set_hide_travel(bool value)
{
  layer_range_vis_->setHideTravel(value);
  DisplayGcodeLayerRange();
}

void GcodeVisualizationWidget::set_color_method()
{
  auto method = static_cast<ColorMethod>(color_method_->currentIndex());
  layer_range_vis_->setColorMethod(method);
  DisplayGcodeLayerRange();
}

void GcodeVisualizationWidget::pick_color_click()
{
  layer_color_ = QColorDialog::getColor(layer_color_, this, "Layer Color");
  layer_range_vis_->setColor(
      create_color_rgba(layer_color_.red(), layer_color_.green(),
                        layer_color_.blue(), layer_color_.alpha()));

  DisplayGcodeLayerRange();
}

void GcodeVisualizationWidget::set_display_style(int value)
{
  auto display_style = static_cast<DisplayStyle>(value);
  layer_range_vis_->setDisplayStyle(display_style);
  DisplayGcodeLayerRange();
}

void GcodeVisualizationWidget::SetToolpath(std::shared_ptr<Toolpath> toolpath)
{
  layer_range_vis_->setLayers(toolpath->layers());
  int n_layers = layer_range_vis_->nLayers() - 1;
  // layer_range_vis_->update(0, n_layers, true);

  layer_slider_->setRangeBounds(0, n_layers);
  layer_slider_->setLowerValue(0);
  layer_slider_->setUpperValue(n_layers);
}

void GcodeVisualizationWidget::DisplayGcodeLayerRange()
{
  int lower = layer_slider_->getLowerValue();
  int upper = layer_slider_->getUpperValue();

  layer_range_vis_->update(lower, upper);
}

}  // namespace gcode_rviz
