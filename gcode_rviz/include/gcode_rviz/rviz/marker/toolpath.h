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
class LineList;

class ToolpathMarker
{
public:
  ToolpathMarker(GcodeDisplay* owner, rviz::DisplayContext* context,
                 Ogre::SceneNode* parent_node);

  virtual ~ToolpathMarker();

  void setMessage(const gcode_msgs::Toolpath& message);
  void setMessage(const gcode_msgs::ToolpathConstPtr& message);

  inline int32_t getID() { return message_->id; }

  virtual void setPosition(const Ogre::Vector3& position);
  virtual void setOrientation(const Ogre::Quaternion& orientation);
  const Ogre::Vector3& getPosition() const;
  const Ogre::Quaternion& getOrientation() const;

protected:
  bool transform(const gcode_msgs::ToolpathConstPtr& message,
                 Ogre::Vector3& pos, Ogre::Quaternion& orient);
  void onNewMessage(const gcode_msgs::ToolpathConstPtr& old_message,
                    const gcode_msgs::ToolpathConstPtr& new_message);

  GcodeDisplay* owner_;
  rviz::DisplayContext* context_;
  Ogre::SceneNode* scene_node_;

  gcode_msgs::ToolpathConstPtr message_;
  LineList* line_list_;
};
typedef boost::shared_ptr<ToolpathMarker> ToopathMarkerPtr;

}  // namespace gcode_rviz

#endif  // RVIZ_GCODE_TOOLPATH_H
