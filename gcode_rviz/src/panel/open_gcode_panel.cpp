#include <QBoxLayout>
#include <QFileDialog>

#include <gcode_core/core/interpreter.h>
#include <gcode_rviz/panel/open_gcode_panel.h>

namespace gcode_rviz
{
OpenGcodePanel::OpenGcodePanel(QWidget* parent) : rviz::Panel(parent) {}
OpenGcodePanel::~OpenGcodePanel() = default;

void OpenGcodePanel::onInitialize()
{
  QVBoxLayout* main_layout = new QVBoxLayout();
  setLayout(main_layout);

  filepath_line_edit_ = new QLineEdit();
  filepath_line_edit_->setPlaceholderText("gcode filepath");

  browse_button_ = new QPushButton();
  browse_button_->setText(tr("..."));
  browse_button_->setToolTip(tr("Load gcode file"));
  connect(browse_button_, SIGNAL(clicked()), this, SLOT(BrowseButtonClicked()));

  viz_widget_ = new GcodeVisualizationWidget(this);

  QHBoxLayout* file_layout = new QHBoxLayout();
  file_layout->addWidget(filepath_line_edit_);
  file_layout->addWidget(browse_button_);

  main_layout->addWidget(new QLabel("Gcode file:", nullptr));
  main_layout->addLayout(file_layout);
  main_layout->addWidget(viz_widget_);
  main_layout->addStretch();
}

void OpenGcodePanel::BrowseButtonClicked()
{
  QString filepath = QFileDialog::getOpenFileName(this, "Open Gcode", "/home/",
                                                  "Gcode Files (*.gcode)");

  if (!filepath.isEmpty())
  {
    filepath_line_edit_->setText(filepath);
    std::string std_filepath = filepath.toStdString();

    toolpath_ = std::make_shared<gcode_core::Toolpath>();
    std::shared_ptr<gcode_core::GcodeInterpreter> interpreter =
        std::make_shared<gcode_core::GcodeInterpreter>();
    interpreter->parseGcode(std_filepath, *toolpath_);

    viz_widget_->setToolpath(toolpath_);
  }
}

}  // namespace gcode_rviz

#include <class_loader/class_loader.hpp>

CLASS_LOADER_REGISTER_CLASS(gcode_rviz::OpenGcodePanel, rviz::Panel)
