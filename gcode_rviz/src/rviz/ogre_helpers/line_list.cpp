#include <gcode_rviz/rviz/ogre_helpers/line_list.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>

namespace gcode_rviz
{

LineList::LineList(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node)
  : rviz::Object(manager), n_lines_(0)
{
  if (!parent_node)
  {
    parent_node = scene_manager_->getRootSceneNode();
  }
  manual_object_ = scene_manager_->createManualObject();
  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "LineListMaterial" << count++;

  manual_object_material_ = Ogre::MaterialManager::getSingleton().create(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  manual_object_material_->setReceiveShadows(false);
  manual_object_material_->getTechnique(0)->setLightingEnabled(true);
  manual_object_material_->getTechnique(0)->getPass(0)->setDiffuse(0, 0, 0, 0);
  manual_object_material_->getTechnique(0)->getPass(0)->setAmbient(1, 1, 1);

  scene_node_->attachObject(manual_object_);
}

LineList::~LineList()
{
  if (scene_node_->getParentSceneNode())
  {
    scene_node_->getParentSceneNode()->removeChild(scene_node_);
  }
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);
  Ogre::MaterialManager::getSingleton().remove(
      manual_object_material_->getName());
}

void LineList::setLines(const OgreLineList& lines,
                        const Ogre::ColourValue& colour)
{
  manual_object_->clear();
  manual_object_->begin(manual_object_material_->getName(),
                        Ogre::RenderOperation::OT_LINE_LIST);
  for (std::size_t i = 0; i < lines.size(); ++i)
  {
    manual_object_->position(lines[i].first);
    manual_object_->colour(colour);
    manual_object_->position(lines[i].second);
    manual_object_->colour(colour);
  }
  manual_object_->end();
  n_lines_ = lines.size();
}

void LineList::setLines(const OgreLineList& lines,
                        const OgreColourList& colours)
{
  manual_object_->clear();
  manual_object_->begin(manual_object_material_->getName(),
                        Ogre::RenderOperation::OT_LINE_LIST);
  for (std::size_t i = 0; i < lines.size(); ++i)
  {
    manual_object_->position(lines[i].first);
    manual_object_->colour(colours[i]);
    manual_object_->position(lines[i].second);
    manual_object_->colour(colours[i]);
  }
  manual_object_->end();
  n_lines_ = lines.size();
}

void LineList::setPosition(const Ogre::Vector3& position)
{
  scene_node_->setPosition(position);
}

void LineList::setOrientation(const Ogre::Quaternion& orientation)
{
  scene_node_->setOrientation(orientation);
}

void LineList::setScale(const Ogre::Vector3& scale)
{
  scene_node_->setScale(scale);
}

void LineList::setColor(float r, float g, float b, float a)
{
  setColor(Ogre::ColourValue(r, g, b, a));
}

void LineList::setColor(const Ogre::ColourValue& c)
{
  manual_object_material_->getTechnique(0)->setAmbient(c * 0.5);
  manual_object_material_->getTechnique(0)->setDiffuse(c);

  if (c.a < 0.9998)
  {
    manual_object_material_->getTechnique(0)->setSceneBlending(
        Ogre::SBT_TRANSPARENT_ALPHA);
    manual_object_material_->getTechnique(0)->setDepthWriteEnabled(false);
  }
  else
  {
    manual_object_material_->getTechnique(0)->setSceneBlending(
        Ogre::SBT_REPLACE);
    manual_object_material_->getTechnique(0)->setDepthWriteEnabled(true);
  }
}

const Ogre::Vector3& LineList::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& LineList::getOrientation()
{
  return scene_node_->getOrientation();
}

void LineList::setUserData(const Ogre::Any& data)
{
  manual_object_->getUserObjectBindings().setUserAny(data);
}

}  // namespace gcode_rviz
