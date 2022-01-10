#ifndef OPEN_GCODE_PANEL_H
#define OPEN_GCODE_PANEL_H

#include <rviz/panel.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QTextEdit>

#include "gcode_rviz/widget/gcode_visualization_widget.h"
#include "gcode_core/core/gcode.h"

using namespace gcode_core; 

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
    // void SimplifyDouglasPeucker(EigenSTL::vector_Vector3d& path, double tolerance);

private Q_SLOTS:
    void BrowseButtonClicked();

protected:
    QPushButton* browse_button_;
    QLineEdit* filepath_line_edit_;

    GcodeVisualizationWidget* viz_widget_;

    GcodeBasePtr gcode_;
    rviz_visual_tools::RvizVisualToolsPtr rvt_;
};

} //namespace gcode_rviz

#endif // OPEN_GCODE_PANEL_H
