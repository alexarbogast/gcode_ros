#include <gcode_msgs/Toolpath.h>
#include <gcode_rviz/rviz/gcode_display.h>
#include <gcode_core/core/conversions.h>

#include <rviz/display_context.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/status_property.h>

namespace gcode_rviz
{
GcodeDisplay::GcodeDisplay() : rviz::Display(), tf_filter_(nullptr)
{
  gcode_topic_property_ = new rviz::RosTopicProperty(
      "Gcode Topic", "visualization_gcode",
      QString::fromStdString(
          ros::message_traits::datatype<gcode_msgs::Toolpath>()),
      "gcode_msgs::Toolpath topic to subscribe to.", this,
      &GcodeDisplay::updateTopic);

  queue_size_property_ =
      new rviz::IntProperty("Queue Size", 100,
                            "Advanced: Set the size of the incoming gcode "
                            "message queue.",
                            this, &GcodeDisplay::updateQueueSize);
  queue_size_property_->setMin(0);
}

void GcodeDisplay::onInitialize()
{
  tf_filter_ = new tf2_ros::MessageFilter<gcode_msgs::Toolpath>(
      *context_->getTF2BufferPtr(), fixed_frame_.toStdString(),
      queue_size_property_->getInt(), update_nh_);

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&GcodeDisplay::incomingToolpath,
                                           this, boost::placeholders::_1));
}

GcodeDisplay::~GcodeDisplay()
{
  if (initialized())
  {
    GcodeDisplay::unsubscribe();
    clearMarkers();
    delete tf_filter_;
  }
}

void GcodeDisplay::reset()
{
  Display::reset();
  // clearMarkers();
}

void GcodeDisplay::deleteToolpath(const ToolpathID& id)
{
  deleteToolpathStatus(id);
  deleteToolpathInternal(id);
}

void GcodeDisplay::deleteAllToolpaths()
{
  std::vector<ToolpathID> to_delete;
  M_IDToToolpathMarker::iterator marker_it = markers_.begin();
  for (; marker_it != markers_.end(); ++marker_it)
  {
    to_delete.push_back(marker_it->first);
  }

  for (std::vector<ToolpathID>::iterator it = to_delete.begin();
       it != to_delete.end(); ++it)
  {
    deleteToolpath(*it);
  }
}

void GcodeDisplay::clearMarkers()
{
  markers_.clear();
  if (tf_filter_)  // also clear messages in pipeline
    tf_filter_->clear();
}

void GcodeDisplay::onEnable() { subscribe(); }
void GcodeDisplay::onDisable()
{
  unsubscribe();
  reset();
}

void GcodeDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  std::string gcode_topic = gcode_topic_property_->getTopicStd();
  if (gcode_topic.empty())
  {
    return;
  }

  sub_.unsubscribe();
  try
  {
    sub_.subscribe(update_nh_, gcode_topic, queue_size_property_->getInt());
    setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(rviz::StatusProperty::Error, "Topic",
              QString("Error subscribing: ") + e.what());
  }
}

void GcodeDisplay::unsubscribe() { sub_.unsubscribe(); }

void GcodeDisplay::updateTopic()
{
  onDisable();
  onEnable();
}

void GcodeDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize((uint32_t)queue_size_property_->getInt());
  subscribe();
}

void GcodeDisplay::incomingToolpath(
    const gcode_msgs::Toolpath::ConstPtr& toolpath)
{
  boost::mutex::scoped_lock lock(queue_mutex_);
  message_queue_.push_back(toolpath);
}

void GcodeDisplay::setTopic(const QString& topic, const QString& datatype)
{
  gcode_topic_property_->setString(topic);
}

}  // namespace gcode_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gcode_rviz::GcodeDisplay, rviz::Display)
