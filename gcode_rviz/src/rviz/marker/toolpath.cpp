#include <gcode_rviz/rviz/marker/toolpath.h>
#include <gcode_rviz/rviz/gcode_display.h>
#include <rviz/display_context.h>

#include <unordered_map>

// #include <gcode_rviz/rviz/ogre_helpers/line_list.h>

#include <rviz/properties/enum_property.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>

namespace gcode_rviz
{
const Ogre::ColourValue CARBON = Ogre::ColourValue(0.2, 0.2, 0.2, 1.0);
const Ogre::ColourValue FRIENDLY_FOX = Ogre::ColourValue(0.87, 0.36, 0.12, 1.0);
const Ogre::ColourValue GREEN_BLUE = Ogre::ColourValue(0.23, 0.72, 0.58, 1.0);
const Ogre::ColourValue BROCADE = Ogre::ColourValue(0.55, 0.52, 0.76, 1.0);
const Ogre::ColourValue MELTED_BUTTER = Ogre::ColourValue(1.0, 0.81, 0.34, 1.0);
const Ogre::ColourValue GUNMETAL = Ogre::ColourValue(0.14, 0.18, 0.25, 1.0);
const Ogre::ColourValue SATIN_GOLD = Ogre::ColourValue(0.8, 0.64, 0.23, 1.0);
const Ogre::ColourValue TOMATO = Ogre::ColourValue(0.98, 0.34, 0.22, 1.0);
const Ogre::ColourValue KELLY_GREEN = Ogre::ColourValue(0.26, 0.73, 0.16, 1.0);

static std::unordered_map<int, Ogre::ColourValue> ToolColor = {
  { 0, FRIENDLY_FOX }, { 1, GREEN_BLUE }, { 2, BROCADE }, { 3, MELTED_BUTTER },
  { 4, GUNMETAL },     { 5, SATIN_GOLD }, { 6, TOMATO },  { 7, KELLY_GREEN }
};

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
  color = CARBON;
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
      color = FRIENDLY_FOX;
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
    color = CARBON;
    return;
  }

  GcodeDisplay::ColorMethod color_method =
      GcodeDisplay::ColorMethod(owner_->color_method_property_->getOptionInt());
  switch (color_method)
  {
    case GcodeDisplay::ColorMethod::BY_TOOL:
      color = ToolColor[move.tool % ToolColor.size()];
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
