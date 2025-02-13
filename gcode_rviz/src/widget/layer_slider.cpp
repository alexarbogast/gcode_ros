#include <gcode_rviz/widget/layer_slider.h>

#include <QHBoxLayout>

namespace gcode_rviz
{
LayerSliderWidget::LayerSliderWidget(QWidget* parent)
  : QWidget(parent)
{
  label_ = new QLabel(this);
  slider_ = new QSlider(Qt::Horizontal, this);
  spinbox_ = new QSpinBox(this);

  QHBoxLayout* layout = new QHBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(label_);
  layout->addWidget(slider_, 1);
  layout->addWidget(spinbox_);
  setLayout(layout);

  setRange(0, 100);

  connect(slider_, &QSlider::valueChanged, this, [this](int value) {
    spinbox_->blockSignals(true);
    spinbox_->setValue(value);
    spinbox_->blockSignals(false);
  });
  connect(slider_, &QSlider::valueChanged, this,
          &LayerSliderWidget::valueChanged);
  connect(slider_, &QSlider::sliderReleased, this,
          &LayerSliderWidget::editingFinished);

  connect(spinbox_, qOverload<int>(&QSpinBox::valueChanged), slider_,
          &QSlider::setValue);
  connect(spinbox_, qOverload<int>(&QSpinBox::valueChanged), this,
          &LayerSliderWidget::editingFinished);
}

void LayerSliderWidget::setLabel(const QString& value)
{
  label_->setText(value);
}

void LayerSliderWidget::setMinimum(int value)
{
  slider_->setMinimum(value);
  spinbox_->setMinimum(value);
}

void LayerSliderWidget::setMaximum(int value)
{
  slider_->setMaximum(value);
  spinbox_->setMaximum(value);
}

void LayerSliderWidget::setRange(int min, int max)
{
  slider_->setRange(min, max);
  spinbox_->setRange(min, max);
}

void LayerSliderWidget::setValue(int value)
{
  slider_->blockSignals(true);
  spinbox_->blockSignals(true);

  spinbox_->setValue(value);
  slider_->setValue(value);

  slider_->blockSignals(false);
  spinbox_->blockSignals(false);

  Q_EMIT valueChanged(value);
}

LayerRangeWidget::LayerRangeWidget(QWidget* parent)
  : QWidget(parent)
{
  min_layer_slider_ = new LayerSliderWidget(this);
  min_layer_slider_->setLabel(tr("min layer: "));
  max_layer_slider_ = new LayerSliderWidget(this);
  max_layer_slider_->setLabel(tr("max layer: "));

  QVBoxLayout* main_layout = new QVBoxLayout();
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->addWidget(min_layer_slider_);
  main_layout->addWidget(max_layer_slider_);
  setLayout(main_layout);

  connect(min_layer_slider_, &LayerSliderWidget::valueChanged, this,
          &LayerRangeWidget::setLowerValue);
  connect(max_layer_slider_, &LayerSliderWidget::valueChanged, this,
          &LayerRangeWidget::setUpperValue);

  connect(min_layer_slider_, &LayerSliderWidget::editingFinished, this,
          [this]() { Q_EMIT editingFinished(); });
  connect(max_layer_slider_, &LayerSliderWidget::editingFinished, this,
          [this]() { Q_EMIT editingFinished(); });
}

void LayerRangeWidget::setUpperValue(int value)
{
  int value_int = std::max(value, min_layer_slider_->value());
  if (value_int != max_layer_slider_->value())
  {
    max_layer_slider_->setValue(value_int);
  }
}

void LayerRangeWidget::setLowerValue(int value)
{
  int value_int = std::min(value, max_layer_slider_->value());
  if (value_int != min_layer_slider_->value())
  {
    min_layer_slider_->setValue(value_int);
  }
}

void LayerRangeWidget::setRangeBounds(int lower, int upper)
{
  min_layer_slider_->setRange(lower, upper);
  max_layer_slider_->setRange(lower, upper);
}

}  // namespace gcode_rviz
