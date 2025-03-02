#ifndef RVIZ_GCODE_COLOR_H
#define RVIZ_GCODE_COLOR_H

#include <OgreColourValue.h>

namespace gcode_rviz
{
class ColorPalette
{
public:
  static const Ogre::ColourValue CARBON;
  static const Ogre::ColourValue FEROCIOUS_FOX;
  static const Ogre::ColourValue GREEN_BLUE;
  static const Ogre::ColourValue BROCADE;
  static const Ogre::ColourValue MELTED_BUTTER;
  static const Ogre::ColourValue GUNMETAL;
  static const Ogre::ColourValue SATIN_GOLD;
  static const Ogre::ColourValue TOMATO;
  static const Ogre::ColourValue KELLY_GREEN;

  static const std::array<Ogre::ColourValue, 8> colors;

  static Ogre::ColourValue getColor(int index);
};

}  // namespace gcode_rviz

#endif  // RVIZ_GCODE_COLOR_H
