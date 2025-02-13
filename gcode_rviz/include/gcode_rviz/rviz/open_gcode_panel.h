#ifndef OPEN_GCODE_PANEL_H
#define OPEN_GCODE_PANEL_H

#include <rviz/panel.h>

#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QTextEdit>

#include <gcode_rviz/widget/gcode_visualization_widget.h>

namespace gcode_rviz
{
class OpenGcodePanel : public rviz::Panel
{
  Q_OBJECT
public:
  OpenGcodePanel(QWidget* parent = nullptr);
  ~OpenGcodePanel() override;

  virtual void onInitialize() override;
  void DisplayToolpath() const;

private Q_SLOTS:
  void BrowseButtonClicked();

protected:
  QPushButton* browse_button_;
  QLineEdit* filepath_line_edit_;

  GcodeVisualizationWidget* viz_widget_;
  gcode_core::ToolpathPtr toolpath_;
};

}  // namespace gcode_rviz

#endif  // OPEN_GCODE_PANEL_H
