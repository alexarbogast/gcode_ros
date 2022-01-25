#include <QGroupBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QColorDialog>

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

    // frame selection
    QHBoxLayout* frame_layout = new QHBoxLayout();
    gcode_frame_ = new QLineEdit(tr("world"));

    frame_layout->addWidget(new QLabel(tr("frame: ")));
    frame_layout->addWidget(gcode_frame_);

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

    //display style
    display_style_ = new QComboBox();
    display_style_->addItems({"lines", "cylinders"});
    
    QHBoxLayout* display_style_layout = new QHBoxLayout();
    display_style_layout->addWidget(new QLabel(tr("display style: ")));
    display_style_layout->addWidget(display_style_);

    // color method
    layer_color_ = QColor(221, 64, 58);
    color_method_ = new QComboBox();
    color_method_->addItems({"random by layer", "random by bead", "uniform layers"});
    pick_color_button_ = new QPushButton(tr("..."));
    pick_color_button_->setToolTip(tr("pick color"));

    QHBoxLayout* color_method_layout = new QHBoxLayout();
    color_method_layout->addWidget(new QLabel(tr("color method: ")));
    color_method_layout->addWidget(color_method_);
    color_method_layout->addWidget(pick_color_button_);
    
    group_box_layout->addLayout(frame_layout);
    group_box_layout->addLayout(min_layer_bl);
    group_box_layout->addLayout(max_layer_bl);
    group_box_layout->addWidget(new QLabel(tr("Apperance: ")));
    group_box_layout->addLayout(line_width_layout);
    group_box_layout->addLayout(display_style_layout);
    group_box_layout->addLayout(color_method_layout);

    group_box_layout->setAlignment(Qt::AlignTop);

    main_layout->addWidget(group_box);

    connect(gcode_frame_, &QLineEdit::textChanged, 
            this, &GcodeVisualizationWidget::set_gcode_frame);

    connect(min_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged), 
            this, &GcodeVisualizationWidget::set_min_layer);
    connect(max_layer_spinbox_, qOverload<int>(&QSpinBox::valueChanged),
            this, &GcodeVisualizationWidget::set_max_layer);

    connect(min_layer_slider_, &QSlider::valueChanged, this, &GcodeVisualizationWidget::set_min_layer);
    connect(max_layer_slider_, &QSlider::valueChanged, this, &GcodeVisualizationWidget::set_max_layer);

    connect(line_width_, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &GcodeVisualizationWidget::DisplayGcodeLayerRange);
    connect(display_style_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &GcodeVisualizationWidget::DisplayGcodeLayerRange);
    connect(pick_color_button_, &QPushButton::clicked,
            this, &GcodeVisualizationWidget::pick_color_click);   
    connect(color_method_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &GcodeVisualizationWidget::DisplayGcodeLayerRange);
}

GcodeVisualizationWidget::~GcodeVisualizationWidget() {}

// slots
void GcodeVisualizationWidget::set_gcode_frame()
{
    rvt_->setBaseFrame(gcode_frame_->text().toStdString());
    DisplayGcodeLayerRange();
}

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

void GcodeVisualizationWidget::pick_color_click()
{
    layer_color_ = QColorDialog::getColor(layer_color_, this, "Layer Color");
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
    if (gcode_)
    {
        rvt_->deleteAllMarkers();
        rvt_->trigger();
        int lower = min_layer_slider_->value();
        int upper = max_layer_slider_->value(); 

        //  ========== color ===========
        int color_method = color_method_->currentIndex();

        std::vector<std_msgs::ColorRGBA> colors;
        for (int layer_ind = lower; layer_ind <= upper; layer_ind++)
        {
            std_msgs::ColorRGBA layer_color;

            if (color_method == 2) // uniform layer
            {
                layer_color.r = (double)layer_color_.red()    / 255;
                layer_color.g = (double)layer_color_.green()  / 255;
                layer_color.b = (double)layer_color_.blue()   / 255;
                layer_color.a = (double)layer_color_.alpha()  / 255;
            }
            else
                layer_color = rvt_->createRandColor();

            for (auto bead : gcode_->toolpath()[layer_ind])
            {
                std_msgs::ColorRGBA bead_color;
                if (color_method == 1) // random by bead 
                {
                    bead_color = rvt_->createRandColor();
                }
                else
                    bead_color = layer_color;

                for (auto cmd : *bead)
                {   
                    colors.push_back(bead_color);
                }
            }
        }

        //  ========== display_style ===========
        geometry_msgs::Vector3 scale;
        double line_width_mm = line_width_->value() / 1000;
        scale.x = line_width_mm;
        scale.y = line_width_mm;
        scale.z = line_width_mm;

        int display_style = display_style_->currentIndex();
        if (display_style == 0) // lines
        {
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
                    }
                }
            }

            rvt_->publishLines(a_points, b_points, colors, scale);
        }
        else if (display_style == 1) // cylinders
        {
            EigenSTL::vector_Vector3d path;
            for (int layer_ind = lower; layer_ind <= upper; layer_ind++)
                for (auto& bead : gcode_->toolpath()[layer_ind])
                    for (auto& cmd : *bead)
                        path.push_back(cmd->translation() / 1000);

            rvt_->publishPath(path, colors, line_width_mm);
        }

        rvt_->trigger();
    }
}

} // namespace gcode_rviz