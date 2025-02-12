#include <gcode_rviz/widget/layer_slider.h>

#include <QHBoxLayout>
#include <QLabel>

namespace gcode_rviz
{
void LayerRangeModel::setMinValue(const QVariant& value)
{
  if (min_value_ != value)
  {
    min_value_ = value;
    Q_EMIT minValueChanged(min_value_);
  }
}

void LayerRangeModel::setMaxValue(const QVariant& value)
{
  if (max_value_ != value)
  {
    max_value_ = value;
    Q_EMIT maxValueChanged(max_value_);
  }
}

LayerSliderWidget::LayerSliderWidget(QWidget* parent)
{
  model_ = QSharedPointer<LayerRangeModel>::create();

  min_layer_slider_ = new QSlider(Qt::Horizontal);
  max_layer_slider_ = new QSlider(Qt::Horizontal);
  min_layer_spinbox_ = new QSpinBox();
  max_layer_spinbox_ = new QSpinBox();

  QHBoxLayout* min_layer_bl = new QHBoxLayout();
  min_layer_bl->addWidget(new QLabel(tr("min layer: ")));
  min_layer_bl->addWidget(min_layer_slider_);
  min_layer_bl->addWidget(min_layer_spinbox_);

  QHBoxLayout* max_layer_bl = new QHBoxLayout();
  max_layer_bl->addWidget(new QLabel(tr("max layer: ")));
  max_layer_bl->addWidget(max_layer_slider_);
  max_layer_bl->addWidget(max_layer_spinbox_);

  QVBoxLayout* main_layout = new QVBoxLayout();
  main_layout->addLayout(min_layer_bl);
  main_layout->addLayout(max_layer_bl);
  setLayout(main_layout);

  connect(min_layer_slider_, &QSlider::sliderReleased, this,
          [this]() { model_->setMinValue(min_layer_slider_->value()); });
  connect(max_layer_slider_, &QSlider::sliderReleased, this,
          [this]() { model_->setMaxValue(max_layer_slider_->value()); });
  connect(min_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
          [this](int value) { model_->setMinValue(value); });
  connect(max_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
          [this](int value) { model_->setMaxValue(value); });

  connect(model_.data(), &LayerRangeModel::minValueChanged, this,
          &LayerSliderWidget::onMinSliderChanged);
  connect(model_.data(), &LayerRangeModel::minValueChanged, this,
          &LayerSliderWidget::onMinSpinboxChanged);
  connect(model_.data(), &LayerRangeModel::maxValueChanged, this,
          &LayerSliderWidget::onMaxSliderChanged);
  connect(model_.data(), &LayerRangeModel::maxValueChanged, this,
          &LayerSliderWidget::onMaxSpinboxChanged);

  // signal that triggers when either min or max values changes
  connect(model_.data(), &LayerRangeModel::minValueChanged, this,
          [this]() { Q_EMIT valueChanged(); });
  connect(model_.data(), &LayerRangeModel::maxValueChanged, this,
          [this]() { Q_EMIT valueChanged(); });
}

void LayerSliderWidget::setUpperValue(int value) { model_->setMaxValue(value); }

void LayerSliderWidget::setLowerValue(int value) { model_->setMinValue(value); }

void LayerSliderWidget::setRangeBounds(int lower, int upper)
{
  min_layer_slider_->setRange(lower, upper);
  min_layer_spinbox_->setRange(lower, upper);
  max_layer_slider_->setRange(lower, upper);
  max_layer_spinbox_->setRange(lower, upper);
}

void LayerSliderWidget::onMinSliderChanged(const QVariant& value)
{
  int value_int = std::min(value.toInt(), max_layer_slider_->value());
  if (value_int != min_layer_slider_->value())
  {
    min_layer_slider_->setValue(value_int);
  }
}

void LayerSliderWidget::onMaxSliderChanged(const QVariant& value)
{
  int value_int = std::max(value.toInt(), min_layer_slider_->value());
  if (value_int != max_layer_slider_->value())
  {
    max_layer_slider_->setValue(value_int);
  }
}

void LayerSliderWidget::onMinSpinboxChanged(const QVariant& value)
{
  int value_int = std::min(value.toInt(), max_layer_spinbox_->value());
  if (value_int != min_layer_spinbox_->value())
  {
    min_layer_spinbox_->setValue(value_int);
  }
}

void LayerSliderWidget::onMaxSpinboxChanged(const QVariant& value)
{
  int value_int = std::max(value.toInt(), min_layer_spinbox_->value());
  if (value_int != max_layer_spinbox_->value())
  {
    max_layer_spinbox_->setValue(value_int);
  }
}

}  // namespace gcode_rviz
