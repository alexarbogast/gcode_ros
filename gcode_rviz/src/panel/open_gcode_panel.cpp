#include <QHBoxLayout>
#include <QFileDialog>

#include "gcode_core/gcode_reader.h"
#include "gcode_core/flavor_impl/marlin_gcode.h"
#include "gcode_rviz/panel/open_gcode_panel.h"

namespace gcode_rviz
{
OpenGcodePanel::OpenGcodePanel(QWidget* parent) : rviz::Panel(parent) {}
OpenGcodePanel::~OpenGcodePanel() = default;

void OpenGcodePanel::onInitialize()
{
    // initialize rviz_visual_tools
    rvt_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_tools"));
    rvt_->loadMarkerPub();
    rvt_->enableBatchPublishing();

    QGridLayout* layout = new QGridLayout();
    setLayout(layout);

    filepath_line_edit_ = new QLineEdit();
    filepath_line_edit_->setPlaceholderText("gcode filepath");

    browse_button_ = new QPushButton();
    browse_button_->setText(tr("browse"));
    browse_button_->setToolTip(tr("Load gcode file"));
    connect(browse_button_, SIGNAL(clicked()), this, SLOT(BrowseButtonClicked()));
    
    viz_widget_ = new GcodeVisualizationWidget(this);

    QHBoxLayout* file_layout = new QHBoxLayout();
    file_layout->addWidget(new QLabel("Gcode file:", nullptr));
    file_layout->addWidget(filepath_line_edit_);
    file_layout->addWidget(browse_button_);
    file_layout->addWidget(viz_widget_);

    layout->addLayout(file_layout, 0, 0);   
}

void OpenGcodePanel::BrowseButtonClicked()
{
    filepath_line_edit_->setText(QFileDialog::getOpenFileName(this, "Open Gcode",
        "/home/", "Image Files (*.gcode)"));

    if (!filepath_line_edit_->text().isEmpty())
    {
        std::string filepath = filepath_line_edit_->text().toStdString();
        
        gcode_ = std::make_shared<MarlinGcode>();
        GcodeReader::ParseGcode(filepath, *gcode_);
    }

    viz_widget_->DisplayGcode(gcode_);
}

} // namespace gcode_rviz

#include <class_loader/class_loader.hpp>

CLASS_LOADER_REGISTER_CLASS(gcode_rviz::OpenGcodePanel, rviz::Panel)