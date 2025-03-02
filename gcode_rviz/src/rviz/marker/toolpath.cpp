#include <gcode_rviz/rviz/gcode_display.h>
#include <gcode_rviz/rviz/marker/toolpath.h>
#include <gcode_rviz/rviz/color.h>

#include <rviz/display_context.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>

namespace gcode_rviz
{
ToolpathMarker::ToolpathMarker(GcodeDisplay* owner,
                               rviz::DisplayContext* context,
                               Ogre::SceneNode* parent_node)
  : owner_(owner)
  , context_(context)
  , scene_node_(parent_node->createChildSceneNode())
  , manual_object_(nullptr)
{
  manual_object_ = context_->getSceneManager()->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
  scene_node_->setScale(Ogre::Vector3(0.001));
}

ToolpathMarker::~ToolpathMarker()
{
  manual_object_->clear();
  context_->getSceneManager()->destroyManualObject(manual_object_);
  manual_object_ = nullptr;

  context_->getSceneManager()->destroySceneNode(scene_node_);
}

void ToolpathMarker::setMessage(const gcode_msgs::Toolpath& message)
{
  gcode_msgs::ToolpathConstPtr message_ptr(new gcode_msgs::Toolpath(message));
  setMessage(message_ptr);
}

void ToolpathMarker::setMessage(const gcode_msgs::ToolpathConstPtr& message)
{
  message_ = message;

  Ogre::Vector3 pos;
  Ogre::Quaternion orient;

  if (!transform(message_, pos, orient))
  {
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);

  manual_object_->clear();
  manual_object_->estimateVertexCount(message_->moves.size());
  manual_object_->begin(
      "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST,
      Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);

  Ogre::ColourValue layer_color;
  getLayerColor(layer_color);

  n_lines_ = message_->moves.size() - 1;
  for (std::size_t i = 0; i < n_lines_; ++i)
  {
    const geometry_msgs::Point& p_start = message_->moves[i].pose.position;
    const geometry_msgs::Point& p_end = message->moves[i + 1].pose.position;

    if (message->moves[i + 1].type == gcode_msgs::Move::TRAVEL)
    {
      if (owner_->hide_travel_property_->getBool())
        continue;
    }

    Ogre::ColourValue move_color = layer_color;
    getMoveColor(message->moves[i + 1], move_color);

    manual_object_->position(Ogre::Vector3(p_start.x, p_start.y, p_start.z));
    manual_object_->colour(move_color);
    manual_object_->position(Ogre::Vector3(p_end.x, p_end.y, p_end.z));
    manual_object_->colour(move_color);
  }
  manual_object_->end();
  context_->queueRender();
}

void ToolpathMarker::getLayerColor(Ogre::ColourValue& color) const
{
  color = ColorPalette::CARBON;
  GcodeDisplay::ColorMethod color_method =
      GcodeDisplay::ColorMethod(owner_->color_method_property_->getOptionInt());
  switch (color_method)
  {
    case GcodeDisplay::ColorMethod::RANDOM_BY_LAYER:
      color = Ogre::ColourValue(static_cast<float>(rand()) / RAND_MAX,  // R
                                static_cast<float>(rand()) / RAND_MAX,  // G
                                static_cast<float>(rand()) / RAND_MAX   // B
      );
      break;
    case GcodeDisplay::ColorMethod::UNIFORM_LAYERS:
      color = rviz::qtToOgre(owner_->layer_color_property_->getColor());
      break;
    default:
      break;
  }
}

void ToolpathMarker::getMoveColor(const gcode_msgs::Move& move,
                                  Ogre::ColourValue& color) const
{
  if (move.type == gcode_msgs::Move::TRAVEL)
  {
    color = ColorPalette::CARBON;
    return;
  }

  GcodeDisplay::ColorMethod color_method =
      GcodeDisplay::ColorMethod(owner_->color_method_property_->getOptionInt());
  switch (color_method)
  {
    case GcodeDisplay::ColorMethod::BY_TOOL:
      color = ColorPalette::getColor(move.tool);
      break;
    default:
      break;
  }
}

bool ToolpathMarker::transform(const gcode_msgs::ToolpathConstPtr& message,
                               Ogre::Vector3& pos, Ogre::Quaternion& orient)
{
  // for now we transform to the chosen frame
  // in the future support frame selection in gcode_msgs
  if (!context_->getFrameManager()->getTransform(
          owner_->frame_property_->getFrameStd(), message->header.stamp, pos,
          orient))
  {
    std::string error;
    context_->getFrameManager()->transformHasProblems(
        owner_->frame_property_->getFrameStd(), message->header.stamp, error);
    if (owner_)
    {
      owner_->setToolpathStatus(getID(), rviz::StatusProperty::Error, error);
    }
    return false;
  }
  return true;
}

void ToolpathMarker::redraw() { setMessage(this->message_); }

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
