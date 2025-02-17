#ifndef TOOLPATH_H_
#define TOOLPATH_H_

#include <rviz/ogre_helpers/object.h>

#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

namespace Ogre
{
class Any;
}  // namespace Ogre

namespace gcode_rviz
{
typedef std::vector<std::pair<Ogre::Vector3, Ogre::Vector3>> OgreLineList;
typedef std::vector<Ogre::ColourValue> OgreColourList;

class LineList : public rviz::Object
{
public:
  LineList(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node = nullptr);
  virtual ~LineList() override;

  void setLines(const OgreLineList& lines,
                const Ogre::ColourValue& colour = Ogre::ColourValue::Green);
  void setLines(const OgreLineList& lines, const OgreColourList& colours);

  virtual void setPosition(const Ogre::Vector3& position) override;
  virtual void setOrientation(const Ogre::Quaternion& orientation) override;
  virtual void setScale(const Ogre::Vector3& scale) override;

  virtual void setColor(float r, float g, float b, float a) override;
  virtual void setColor(const Ogre::ColourValue& c);

  virtual const Ogre::Vector3& getPosition() override;
  virtual const Ogre::Quaternion& getOrientation() override;

  virtual void setUserData(const Ogre::Any& data) override;

protected:
  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr manual_object_material_;

  std::size_t n_lines_;
};

}  // namespace gcode_rviz

#endif  // TOOLPATH_H_
