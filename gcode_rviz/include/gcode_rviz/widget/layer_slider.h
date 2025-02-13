#ifndef LAYER_SLIDER_H
#define LAYER_SLIDER_H

#include <QLabel>
#include <QSlider>
#include <QSpinBox>

namespace gcode_rviz
{
class LayerSliderWidget : public QWidget
{
  Q_OBJECT
public:
  LayerSliderWidget(QWidget* parent = 0);

  void setMinimum(int value);
  void setMaximum(int value);
  inline int minimum() const { return slider_->minimum(); };
  inline int maximum() const { return slider_->maximum(); };

  void setValue(int value);
  inline int value() const { return slider_->value(); };

  void setRange(int min, int max);
  void setLabel(const QString& value);

Q_SIGNALS:
  void valueChanged(int value);
  void editingFinished();

private:
  QLabel* label_;
  QSlider* slider_;
  QSpinBox* spinbox_;
};

class LayerRangeWidget : public QWidget
{
  Q_OBJECT
public:
  LayerRangeWidget(QWidget* parent = 0);
  virtual ~LayerRangeWidget() = default;

  void setUpperValue(int value);
  inline int getUpperValue() const { return max_layer_slider_->value(); }

  void setLowerValue(int value);
  inline int getLowerValue() const { return min_layer_slider_->value(); }

  void setRangeBounds(int lower, int upper);

Q_SIGNALS:
  void valueChanged();
  void editingFinished();

private:
  LayerSliderWidget* min_layer_slider_;
  LayerSliderWidget* max_layer_slider_;
};

}  // namespace gcode_rviz

#endif  // LAYER_SLIDER_H
