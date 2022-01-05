#include <QHBoxLayout>

#include "gcode_rviz/widget/gcode_visualization_widget.h"
#include "gcode_core/move_command.h"

namespace gcode_rviz
{

GcodeVisualizationWidget::GcodeVisualizationWidget(QWidget* parent)
    : QWidget(parent)
{
    // initialize rviz_visual_tools
    rvt_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_tools"));
    rvt_->loadMarkerPub();
    rvt_->enableBatchPublishing();

    QGridLayout* layout = new QGridLayout();
    setLayout(layout);
}

GcodeVisualizationWidget::~GcodeVisualizationWidget() {}

void GcodeVisualizationWidget::DisplayGcode(GcodeBaseConstPtr gcode)
{
    rvt_->deleteAllMarkers();

    double current_z = 0.0;

    EigenSTL::vector_Vector3d path;
    EigenSTL::vector_Vector3d::iterator layer_begin = path.begin();

    for (const auto& cmd : *gcode)
    {
        if (cmd.GetCommandType() == MoveCommand::GetStaticType())
        {
            Eigen::Vector3d waypoint = cmd.as<MoveCommand>().translation() / 1000; 
            
            path.emplace_back(waypoint);            
        }
    }

    rvt_->publishPath(path, rviz_visual_tools::RAND);
    rvt_->trigger();
}

} // namespace gcode_rviz