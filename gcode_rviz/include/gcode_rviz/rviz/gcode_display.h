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

class GcodeDisplay : public rviz::Display
{
  Q_OBJECT
public:
  GcodeDisplay();
  ~GcodeDisplay() override;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;

  void fixedFrameChanged() override;
  void reset() override;

  void deleteToolpath(int32_t id);
  void deleteAllToolpaths();

  void setToolpathStatus(int32_t id, rviz::StatusLevel level,
                         const std::string& text);
  void deleteToolpathStatus(int32_t id);

  void setTopic(const QString& topic, const QString& datatype) override;

protected:
  void deleteToolpathInternal(int32_t id);

  void onEnable() override;
  void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();

  rviz::RosTopicProperty* gcode_topic_property_;
  rviz::IntProperty* queue_size_property_;

private Q_SLOTS:
  void updateQueueSize();
  void updateTopic();

private:
  typedef std::map<int32_t, ToolpathMarkerPtr> M_IDToToolpathMarker;
  typedef std::vector<gcode_msgs::Toolpath::ConstPtr> V_ToopathMessage;

  void clearMarkers();

  void processMessage(const gcode_msgs::Toolpath::ConstPtr& message);
  void processAdd(const gcode_msgs::Toolpath::ConstPtr& message);
  void processDelete(const gcode_msgs::Toolpath::ConstPtr& message);

  void incomingToolpath(const gcode_msgs::Toolpath::ConstPtr& toolpath);
  void
  failedToolpath(const ros::MessageEvent<gcode_msgs::Toolpath>& toolpath_evt,
                 tf2_ros::FilterFailureReason reason);

  M_IDToToolpathMarker markers_;
  V_ToopathMessage message_queue_;
  boost::mutex queue_mutex_;

  message_filters::Subscriber<gcode_msgs::Toolpath> sub_;
  tf2_ros::MessageFilter<gcode_msgs::Toolpath>* tf_filter_;
};

}  // namespace gcode_rviz

#endif  // GCODE_DISPLAY_H
