#ifndef GCODE_DISPLAY_H
#define GCODE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#endif

#include <gcode_msgs/Toolpath.h>
#include <rviz/display.h>

namespace rviz
{
class RosTopicProperty;
class IntProperty;
}  // namespace rviz

namespace gcode_rviz
{
class ToolpathMarker;

typedef boost::shared_ptr<ToolpathMarker> ToolpathMarkerPtr;
typedef std::pair<std::string, int32_t> ToolpathID;

class GcodeDisplay : public rviz::Display
{
  Q_OBJECT
public:
  GcodeDisplay();
  ~GcodeDisplay() override;

  void onInitialize() override;
  void reset() override;

  void deleteToolpath(const ToolpathID& id);
  void deleteAllToolpaths();

  void setToolpathStatus(const ToolpathID& id, rviz::StatusLevel level,
                         const std::string& text);
  void deleteToolpathStatus(const ToolpathID& id);

  void setTopic(const QString& topic, const QString& datatype) override;

protected:
  void deleteToolpathInternal(const ToolpathID& id);

  void onEnable() override;
  void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();

  rviz::RosTopicProperty* gcode_topic_property_;
  rviz::IntProperty* queue_size_property_;

private Q_SLOTS:
  void updateTopic();
  void updateQueueSize();

private:
  typedef std::map<ToolpathID, ToolpathMarkerPtr> M_IDToToolpathMarker;

  void clearMarkers();
  void incomingToolpath(const gcode_msgs::Toolpath::ConstPtr& toolpath);

  M_IDToToolpathMarker markers_;
  std::vector<gcode_msgs::Toolpath::ConstPtr> message_queue_;
  boost::mutex queue_mutex_;

  message_filters::Subscriber<gcode_msgs::Toolpath> sub_;
  tf2_ros::MessageFilter<gcode_msgs::Toolpath>* tf_filter_;
};

}  // namespace gcode_rviz

#endif  // GCODE_DISPLAY_H
