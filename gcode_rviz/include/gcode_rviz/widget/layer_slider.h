#ifndef LAYER_SLIDER_H
#define LAYER_SLIDER_H

#include <QSlider>
#include <QSpinBox>

namespace gcode_rviz
{
class LayerRangeModel : public QObject
{
  Q_OBJECT
public:
  void setMinValue(const QVariant& value);
  inline const QVariant& getMinValue() const
  {
    return min_value_;
  };

  void setMaxValue(const QVariant& value);
  inline const QVariant& getMaxValue() const
  {
    return max_value_;
  };

Q_SIGNALS:
  void minValueChanged(QVariant& value);
  void maxValueChanged(QVariant& value);

private:
  QVariant min_value_;
  QVariant max_value_;
};

class LayerSliderWidget : public QWidget
{
  Q_OBJECT
public:
  LayerSliderWidget(QWidget* parent = 0);
  virtual ~LayerSliderWidget() = default;

  void setUpperValue(int value);
  inline int getUpperValue() const { return model_->getMaxValue().toInt(); }

  void setLowerValue(int value);
  inline int getLowerValue() const { return model_->getMinValue().toInt(); }

  void setRangeBounds(int lower, int upper);

Q_SIGNALS:
  void valueChanged();

private Q_SLOTS:
  void onMinSpinboxChanged(const QVariant& value);
  void onMaxSpinboxChanged(const QVariant& value);
  void onMinSliderChanged(const QVariant& value);
  void onMaxSliderChanged(const QVariant& value);

private:
  QSharedPointer<LayerRangeModel> model_;

  QSlider* min_layer_slider_;
  QSlider* max_layer_slider_;
  QSpinBox* min_layer_spinbox_;
  QSpinBox* max_layer_spinbox_;
};

}  // namespace gcode_rviz

#endif  // LAYER_SLIDER_H