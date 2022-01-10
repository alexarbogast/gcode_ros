#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>

#include "gcode_rviz/widget/gcode_visualization_widget.h"

namespace gcode_rviz
{

GcodeVisualizationWidget::GcodeVisualizationWidget(QWidget* parent)
    : QWidget(parent)
{
    // initialize rviz_visual_tools
    rvt_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_tools"));
    rvt_->loadMarkerPub();
    rvt_->enableBatchPublishing();

    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    QGroupBox* group_box = new QGroupBox(tr("gcode visual"));
    QVBoxLayout* group_box_layout = new QVBoxLayout();
    group_box->setLayout(group_box_layout);

    // min-max layer box layout
    min_layer_spinbox_ = new QSpinBox();
    max_layer_spinbox_ = new QSpinBox();
    min_layer_slider_ = new QSlider(Qt::Horizontal);
    max_layer_slider_ = new QSlider(Qt::Horizontal);

    QHBoxLayout* min_layer_bl = new QHBoxLayout();
    min_layer_bl->addWidget(new QLabel(tr("min layer: ")));
    min_layer_bl->addWidget(min_layer_slider_);
    min_layer_bl->addWidget(min_layer_spinbox_);

    QHBoxLayout* max_layer_bl = new QHBoxLayout();
    max_layer_bl->addWidget(new QLabel(tr("max layer: ")));
    max_layer_bl->addWidget(max_layer_slider_);
    max_layer_bl->addWidget(max_layer_spinbox_);

    group_box_layout->addLayout(min_layer_bl);
    group_box_layout->addLayout(max_layer_bl);
    group_box_layout->setAlignment(Qt::AlignTop);

    main_layout->addWidget(group_box);

    connect(min_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged), 
            this, &GcodeVisualizationWidget::set_min_layer);

    connect(max_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged),
            this, &GcodeVisualizationWidget::set_max_layer);

    connect(min_layer_slider_, &QSlider::valueChanged, this, &GcodeVisualizationWidget::set_min_layer);
    connect(max_layer_slider_, &QSlider::valueChanged, this, &GcodeVisualizationWidget::set_max_layer); 
}


GcodeVisualizationWidget::~GcodeVisualizationWidget() {}

// slots
void GcodeVisualizationWidget::set_min_layer(int value)
{
    if (value < max_layer_spinbox_->value())
    {
        min_layer_slider_->setValue(value);
        min_layer_spinbox_->setValue(value);
    }
    else 
    {
        min_layer_slider_->setValue(max_layer_spinbox_->value());
        min_layer_spinbox_->setValue(max_layer_spinbox_->value());
    }

    DisplayGcodeLayerRange(min_layer_spinbox_->value(), max_layer_spinbox_->value());
}

void GcodeVisualizationWidget::set_max_layer(int value)
{
    if (value > min_layer_spinbox_->value())
    {
        max_layer_slider_->setValue(value);
        max_layer_spinbox_->setValue(value);
    }
    else 
    {
        max_layer_slider_->setValue(min_layer_spinbox_->value());
        max_layer_spinbox_->setValue(min_layer_spinbox_->value());
    }

    DisplayGcodeLayerRange(min_layer_spinbox_->value(), max_layer_spinbox_->value());
}

void GcodeVisualizationWidget::SetGcode(GcodeBaseConstPtr gcode) 
{
    gcode_ = gcode;

    min_layer_spinbox_->setRange(0, gcode->toolpath().size());
    max_layer_spinbox_->setRange(0, gcode->toolpath().size());
    max_layer_spinbox_->setValue(gcode_->toolpath().size());
}

void GcodeVisualizationWidget::DisplayGcode()
{
    rvt_->deleteAllMarkers();

    EigenSTL::vector_Vector3d path;

    for (auto& layer : gcode_->toolpath())
        for (auto& bead : *layer)
            for (auto& cmd : *bead)
                path.push_back(cmd->translation() / 1000);

    rvt_->publishPath(path, rviz_visual_tools::RAND);
    rvt_->trigger();
}

void GcodeVisualizationWidget::DisplayGcodeLayerRange(unsigned int lower, unsigned int upper)
{
    rvt_->deleteAllMarkers();

    EigenSTL::vector_Vector3d path;

    for (int layer_ind = lower; layer_ind < upper; layer_ind++)
        for (auto& bead : gcode_->toolpath()[layer_ind])
            for (auto& cmd : *bead)
                path.push_back(cmd->translation() / 1000);

    rvt_->publishPath(path, rviz_visual_tools::RAND);
    rvt_->trigger();
}

} // namespace gcode_rviz