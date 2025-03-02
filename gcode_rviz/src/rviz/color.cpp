#include <gcode_rviz/rviz/color.h>

namespace gcode_rviz
{
const Ogre::ColourValue ColorPalette::CARBON =
    Ogre::ColourValue(0.2, 0.2, 0.2, 1.0);
const Ogre::ColourValue ColorPalette::FEROCIOUS_FOX =
    Ogre::ColourValue(0.87, 0.36, 0.12, 1.0);
const Ogre::ColourValue ColorPalette::GREEN_BLUE =
    Ogre::ColourValue(0.23, 0.72, 0.58, 1.0);
const Ogre::ColourValue ColorPalette::BROCADE =
    Ogre::ColourValue(0.55, 0.52, 0.76, 1.0);
const Ogre::ColourValue ColorPalette::MELTED_BUTTER =
    Ogre::ColourValue(1.0, 0.81, 0.34, 1.0);
const Ogre::ColourValue ColorPalette::GUNMETAL =
    Ogre::ColourValue(0.14, 0.18, 0.25, 1.0);
const Ogre::ColourValue ColorPalette::SATIN_GOLD =
    Ogre::ColourValue(0.8, 0.64, 0.23, 1.0);
const Ogre::ColourValue ColorPalette::TOMATO =
    Ogre::ColourValue(0.98, 0.34, 0.22, 1.0);
const Ogre::ColourValue ColorPalette::KELLY_GREEN =
    Ogre::ColourValue(0.26, 0.73, 0.16, 1.0);

const std::array<Ogre::ColourValue, 8> ColorPalette::colors = {
  FEROCIOUS_FOX, GREEN_BLUE, BROCADE, MELTED_BUTTER,
  GUNMETAL,      SATIN_GOLD, TOMATO,  KELLY_GREEN
};

Ogre::ColourValue ColorPalette::getColor(int index)
{
  return colors[index % colors.size()];
}

}  // namespace gcode_rviz
