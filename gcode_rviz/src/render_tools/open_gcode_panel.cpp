#include <QHBoxLayout>
#include <QFileDialog>

#include <gcode_rviz/render_tools/open_gcode_panel.h>

namespace gcode_rviz
{
OpenGcodePanel::OpenGcodePanel(QWidget* parent) : rviz::Panel(parent) {}
OpenGcodePanel::~OpenGcodePanel() = default;

void OpenGcodePanel::onInitialize()
{
    filepath_line_edit_ = new QLineEdit;
    filepath_line_edit_->setPlaceholderText("gcode filepath");

    browse_button_ = new QPushButton();
    browse_button_->setText(tr("browse"));
    browse_button_->setToolTip(tr("Load gcode file"));
    connect(browse_button_, SIGNAL(clicked()), this, SLOT(BrowseButtonClicked()));

    auto* layout = new QHBoxLayout;
    layout->addWidget(new QLabel("Gcode file:", nullptr));
    layout->addWidget(filepath_line_edit_);
    layout->addWidget(browse_button_);
    setLayout(layout);
}

void OpenGcodePanel::BrowseButtonClicked()
{
    filepath_line_edit_->setText(QFileDialog::getOpenFileName(this, "Open Gcode",
        "/home/", "Image Files (*.gcode)"));

    if (!filepath_line_edit_->text().isEmpty())
    {
        
    }
}

} // namepspace gcode_rviz

#include <class_loader/class_loader.hpp>
#include <gcode_rviz/render_tools/open_gcode_panel.h>

CLASS_LOADER_REGISTER_CLASS(gcode_rviz::OpenGcodePanel, rviz::Panel)