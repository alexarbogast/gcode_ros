#include <gcode_rviz/rviz/marker/toolpath.h>
#include <gcode_rviz/rviz/gcode_display.h>
#include <rviz/display_context.h>

#include <OgreSceneNode.h>

namespace gcode_rviz
{
ToolpathMarker::ToolpathMarker(GcodeDisplay* owner,
                               rviz::DisplayContext* context,
                               Ogre::SceneNode* parent_node)
  : owner_(owner)
  , context_(context)
  , scene_node_(parent_node->createChildSceneNode())
{
}
}  // namespace gcode_rviz
