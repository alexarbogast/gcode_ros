#include "gcode_rviz/widget/color_list_editor.h"

namespace gcode_rviz
{
ColorListEditor::ColorListEditor(QWidget* widget) : QComboBox(widget)
{
    populateList();
}

QColor ColorListEditor::color() const 
{
    return qvariant_cast<QColor>(itemData(currentIndex(), Qt::DecorationRole));   
}

void ColorListEditor::setColor(const QColor& color)
{
    setCurrentIndex(findData(color, int(Qt::DecorationRole)));
}

void ColorListEditor::populateList()
{
    QStringList color_names = QColor::colorNames();

    for (int i = 0; i < color_names.size(); i++)
    {
        QColor color(color_names[i]);

        insertItem(i, color_names[i]);
        setItemData(i, color, Qt::DecorationRole);
    } 
}

} // namespace gcode_rviz