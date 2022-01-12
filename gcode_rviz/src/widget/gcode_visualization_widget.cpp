#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>

#include <eigen_conversions/eigen_msg.h>
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
    rvt_->enableFrameLocking();

    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    QGroupBox* group_box = new QGroupBox(tr("Gcode Visual"));
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

    // line width layout
    QHBoxLayout* line_width_layout = new QHBoxLayout();
    line_width_ = new QDoubleSpinBox();
    line_width_->setMinimum(0.001);
    line_width_->setDecimals(2);
    line_width_->setSingleStep(0.5);
    line_width_->setValue(1);
    line_width_layout->addWidget(new QLabel(tr("line width (mm): ")));
    line_width_layout->addWidget(line_width_);

    color_list_editor_ = new ColorListEditor();

    group_box_layout->addLayout(min_layer_bl);
    group_box_layout->addLayout(max_layer_bl);
    group_box_layout->addWidget(new QLabel(tr("Apperance: ")));
    group_box_layout->addLayout(line_width_layout);
    group_box_layout->addWidget(color_list_editor_);

    group_box_layout->setAlignment(Qt::AlignTop);

    main_layout->addWidget(group_box);

    connect(min_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged), 
            this, &GcodeVisualizationWidget::set_min_layer);

    connect(max_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged),
            this, &GcodeVisualizationWidget::set_max_layer);

    connect(min_layer_slider_, &QSlider::valueChanged, this, &GcodeVisualizationWidget::set_min_layer);
    connect(max_layer_slider_, &QSlider::valueChanged, this, &GcodeVisualizationWidget::set_max_layer);

    connect(line_width_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &GcodeVisualizationWidget::DisplayGcodeLayerRange);
    connect(color_list_editor_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &GcodeVisualizationWidget::DisplayGcodeLayerRange);
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

    DisplayGcodeLayerRange();
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

    DisplayGcodeLayerRange();
}

void GcodeVisualizationWidget::SetGcode(GcodeBaseConstPtr gcode) 
{
    gcode_ = gcode;

    int n_layers = gcode->toolpath().size() - 1;
    min_layer_slider_->setRange(0, n_layers);
    max_layer_slider_->setRange(0, n_layers);
    min_layer_spinbox_->setRange(0, n_layers);
    max_layer_spinbox_->setRange(0, n_layers);
    min_layer_slider_->setValue(0);
    max_layer_slider_->setValue(n_layers);
}

void GcodeVisualizationWidget::DisplayGcodeLayerRange()
{
    DisplayGcodeLines();
}

void GcodeVisualizationWidget::DisplayGcodeLines()
{
    rvt_->deleteAllMarkers();
    rvt_->trigger();
    int lower = min_layer_slider_->value();
    int upper = max_layer_slider_->value();

    std::vector<std_msgs::ColorRGBA> colors;    
    geometry_msgs::Vector3 scale;
    double line_width_mm = line_width_->value() / 1000;
    scale.x = line_width_mm;
    scale.y = line_width_mm;
    scale.z = line_width_mm;

    std::vector<geometry_msgs::Point> a_points, b_points;
    for (int layer_ind = lower; layer_ind <= upper; layer_ind++)
    {
        std_msgs::ColorRGBA layer_color = rvt_->createRandColor();
        for (auto bead : gcode_->toolpath()[layer_ind])
        {
            for (auto it = bead->begin(); it != --bead->end(); ++it)
            {   
                geometry_msgs::Point a_point, b_point;
                tf::pointEigenToMsg((*it)->translation() / 1000, a_point);
                tf::pointEigenToMsg((*(it + 1))->translation() / 1000, b_point);

                a_points.emplace_back(a_point);
                b_points.emplace_back(b_point);
                colors.push_back(layer_color);
            }
        }
    }

    rvt_->publishLines(a_points, b_points, colors, scale);
    rvt_->trigger();
}

//std::vector<std_msgs::ColorRGBA> colors;

    //int r, g, b, a;
    //QColor qcolor = color_list_editor_->color();
    //qcolor.getRgb(&r, &g, &b, &a);
    //
    //std_msgs::ColorRGBA color;
    //color.r = r; color.g = g; color.b = b; color.a = a;

    //for (int layer_ind = lower; layer_ind <= upper; layer_ind++)
    //{
    //    path.clear();
    //    rviz_visual_tools::colors layer_color = rviz_visual_tools::RvizVisualTools::getRandColor();
    //    for (auto& bead : gcode_->toolpath()[layer_ind])
    //    {
    //        for (auto& cmd : *bead)
    //        {
    //            path.push_back(cmd->translation() / 1000);
    //            // colors.push_back(layer_color);
    //        }  
    //    }
    //    rvt_->publishPath(path, layer_color, 2.85/1000);
    //}

    //rvt_->publishPath(path, rvt_->intToRvizColor(color_list_editor_->color().value()), 2.85/1000);
    //rvt_->publishPath(path, colors, 2.85/1000);

} // namespace gcode_rviz