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
  , line_list_(nullptr)
{
}

ToolpathMarker::~ToolpathMarker()
{
  delete line_list_;
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
  if (!line_list_)
  {
    line_list_ = new LineList(context_->getSceneManager(), scene_node_);
  }
  line_list_->setScale(Ogre::Vector3(0.001));

  Ogre::Vector3 pos;
  Ogre::Quaternion orient;

  if (!transform(new_message, pos, orient))
  {
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);

  OgreLineList lines(new_message->moves.size() - 1);
  for (std::size_t i = 1; i < new_message->moves.size(); ++i)
  {
    const geometry_msgs::Point& p_start =
        new_message->moves[i - 1].pose.position;
    const geometry_msgs::Point& p_end = new_message->moves[i].pose.position;

    lines.emplace_back(
        std::make_pair(Ogre::Vector3(p_start.x, p_start.y, p_start.z),
                       Ogre::Vector3(p_end.x, p_end.y, p_end.z)));
  }
  line_list_->setLines(lines);
}

bool ToolpathMarker::transform(const gcode_msgs::ToolpathConstPtr& message,
                               Ogre::Vector3& pos, Ogre::Quaternion& orient)
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
  return true;
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
