#ifndef RVIZ_GCODE_TOOLPATH_H
#define RVIZ_GCODE_TOOLPATH_H

#include <gcode_msgs/Toolpath.h>
#include <OgrePrerequisites.h>

namespace rviz
{
class DisplayContext;
}  // namespace rviz

namespace gcode_rviz
{
class GcodeDisplay;

class ToolpathMarker
{
public:
  ToolpathMarker(GcodeDisplay* owner, rviz::DisplayContext* context,
                 Ogre::SceneNode* parent_node);

protected:
  GcodeDisplay* owner_;
  rviz::DisplayContext* context_;
  Ogre::SceneNode* scene_node_;
};
typedef boost::shared_ptr<ToolpathMarker> ToopathMarkerPtr;

}  // namespace gcode_rviz

#endif  // RVIZ_GCODE_TOOLPATH_H
