#ifndef COLOR_LIST_EDITOR_H
#define COLOR_LIST_EDITOR_H

#include <QComboBox>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace gcode_rviz
{
class ColorListEditor : public QComboBox
{
  Q_OBJECT
  Q_PROPERTY(QColor color READ color WRITE setColor USER true)

public:
  ColorListEditor(QWidget* widget = nullptr);

public:
  QColor color() const;
  void setColor(const QColor& color);

private:
  void populateList();
};

}  // namespace gcode_rviz

#endif  // COLOR_LIST_EDITOR_H
