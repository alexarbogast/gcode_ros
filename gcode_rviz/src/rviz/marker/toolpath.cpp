#include <OgreAny.h>
#include <gcode_rviz/rviz/marker/toolpath.h>
#include <gcode_rviz/rviz/gcode_display.h>
#include <rviz/display_context.h>

#include <gcode_rviz/rviz/ogre_helpers/line_list.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

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

ToolpathMarker::~ToolpathMarker()
{
  context_->getSceneManager()->destroySceneNode(scene_node_);
}

void ToolpathMarker::setMessage(const gcode_msgs::Toolpath& message)
{
  gcode_msgs::ToolpathConstPtr message_ptr(new gcode_msgs::Toolpath(message));
  setMessage(message_ptr);
}

void ToolpathMarker::setMessage(const gcode_msgs::ToolpathConstPtr& message)
{
  gcode_msgs::ToolpathConstPtr old = message_;
  message_ = message;
  onNewMessage(old, message);
}

void ToolpathMarker::onNewMessage(
    const gcode_msgs::ToolpathConstPtr& old_message,
    const gcode_msgs::ToolpathConstPtr& new_message)
{
  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale))
  {
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);
}

bool ToolpathMarker::transform(const gcode_msgs::ToolpathConstPtr& message,
                               Ogre::Vector3& pos, Ogre::Quaternion& orient,
                               Ogre::Vector3& scale)
{
  if (!context_->getFrameManager()->getTransform(message->header, pos, orient))
  {
    std::string error;
    context_->getFrameManager()->transformHasProblems(
        message->header.frame_id, message->header.stamp, error);
    if (owner_)
    {
      owner_->setToolpathStatus(getID(), rviz::StatusProperty::Error, error);
    }
    return false;
  }
  return false;
}

void ToolpathMarker::setPosition(const Ogre::Vector3& position)
{
  scene_node_->setPosition(position);
}
void ToolpathMarker::setOrientation(const Ogre::Quaternion& orientation)
{
  scene_node_->setOrientation(orientation);
}
const Ogre::Vector3& ToolpathMarker::getPosition() const
{
  return scene_node_->getPosition();
}
const Ogre::Quaternion& ToolpathMarker::getOrientation() const
{
  return scene_node_->getOrientation();
}

}  // namespace gcode_rviz
