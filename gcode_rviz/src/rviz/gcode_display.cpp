#include <gcode_rviz/rviz/gcode_display.h>
#include <gcode_rviz/rviz/marker/toolpath.h>

#include <rviz/display_context.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/tf_frame_property.h>

namespace gcode_rviz
{
struct DisplayStyle
{
  enum
  {
    LINES,
    CYLINDERS
  };
};

GcodeDisplay::GcodeDisplay() : rviz::Display()
{
  gcode_topic_property_ = new rviz::RosTopicProperty(
      "Gcode Topic", "/visualization_gcode",
      QString::fromStdString(
          ros::message_traits::datatype<gcode_msgs::Toolpath>()),
      "gcode_msgs::Toolpath topic to subscribe to.", this,
      &GcodeDisplay::updateTopic);

  frame_property_ = new rviz::TfFrameProperty(
      "Reference Frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
      "The base reference frame for the gcode.", this, nullptr, true);

  queue_size_property_ =
      new rviz::IntProperty("Queue Size", 100,
                            "Advanced: Set the size of the incoming gcode "
                            "message queue.",
                            this, &GcodeDisplay::updateQueueSize);
  queue_size_property_->setMin(0);

  line_width_property_ = new rviz::FloatProperty(
      "Line Width (mm)", 5.0, "The width of toolpath lines in millimeters",
      this, &GcodeDisplay::updateLineWidth);

  display_style_property_ =
      new rviz::EnumProperty("Display Style", "Lines", "Gcode display style",
                             this, &GcodeDisplay::updateDisplayStyle);
  display_style_property_->addOption("Lines", DisplayStyle::LINES);
  display_style_property_->addOption("Cylinders", DisplayStyle::CYLINDERS);
}

void GcodeDisplay::onInitialize()
{
  frame_property_->setFrameManager(context_->getFrameManager());
}

GcodeDisplay::~GcodeDisplay()
{
  if (initialized())
  {
    GcodeDisplay::unsubscribe();
    clearMarkers();
  }
}

void GcodeDisplay::clearMarkers() { markers_.clear(); }

void GcodeDisplay::onEnable() { subscribe(); }
void GcodeDisplay::onDisable()
{
  unsubscribe();
  reset();
}

void GcodeDisplay::updateQueueSize() { subscribe(); }

void GcodeDisplay::updateTopic()
{
  onDisable();
  onEnable();
}

void GcodeDisplay::updateLineWidth()
{
  // redraw gcode
}

void GcodeDisplay::updateDisplayStyle()
{
  // redraw gcode
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

  try
  {
    toolpath_sub_ =
        update_nh_.subscribe(gcode_topic, queue_size_property_->getInt(),
                             &GcodeDisplay::incomingToolpath, this);
    setStatus(rviz::StatusProperty::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(rviz::StatusProperty::Error, "Topic",
              QString("Error subscribing: ") + e.what());
  }
}

void GcodeDisplay::unsubscribe() { toolpath_sub_.shutdown(); }

void GcodeDisplay::deleteToolpath(int32_t id)
{
  deleteToolpathStatus(id);
  deleteToolpathInternal(id);
}

void GcodeDisplay::deleteToolpathInternal(int32_t id)
{
  M_IDToToolpathMarker::iterator it = markers_.find(id);
  if (it != markers_.end())
  {
    markers_.erase(it);
  }
}

void GcodeDisplay::deleteAllToolpaths()
{
  std::vector<int32_t> to_delete;
  M_IDToToolpathMarker::iterator marker_it = markers_.begin();
  for (; marker_it != markers_.end(); ++marker_it)
  {
    to_delete.push_back(marker_it->first);
  }

  for (std::vector<int32_t>::iterator it = to_delete.begin();
       it != to_delete.end(); ++it)
  {
    deleteToolpath(*it);
  }
}

void GcodeDisplay::setToolpathStatus(int32_t id, rviz::StatusLevel level,
                                     const std::string& text)
{
  setStatusStd(level, std::to_string(id), text);
}

void GcodeDisplay::deleteToolpathStatus(int32_t id)
{
  deleteStatusStd(std::to_string(id));
}

void GcodeDisplay::incomingToolpath(
    const gcode_msgs::Toolpath::ConstPtr& toolpath)
{
  boost::mutex::scoped_lock lock(queue_mutex_);
  message_queue_.push_back(toolpath);
}

void GcodeDisplay::failedToolpath(
    const ros::MessageEvent<gcode_msgs::Toolpath>& toolpath_evt,
    tf2_ros::FilterFailureReason reason)
{
  const gcode_msgs::Toolpath::ConstPtr& toolpath =
      toolpath_evt.getConstMessage();
  if (toolpath->action == gcode_msgs::Toolpath::DELETE ||
      toolpath->action == gcode_msgs::Toolpath::DELETEALL)
  {
    return this->processMessage(toolpath);
  }
  const std::string& authority = toolpath_evt.getPublisherName();
  std::string error = context_->getFrameManager()->discoverFailureReason(
      toolpath->header.frame_id, toolpath->header.stamp, authority, reason);

  setToolpathStatus(toolpath->id, rviz::StatusProperty::Error, error);
}

void GcodeDisplay::processMessage(const gcode_msgs::Toolpath::ConstPtr& message)
{
  switch (message->action)
  {
    case gcode_msgs::Toolpath::ADD:
      processAdd(message);
      break;
    case gcode_msgs::Toolpath::DELETE:
      processDelete(message);
      break;
    case gcode_msgs::Toolpath::DELETEALL:
      deleteAllToolpaths();
      break;

    default:
      ROS_ERROR("Unknown marker action: %d\n", message->action);
  }
}

void GcodeDisplay::processAdd(const gcode_msgs::Toolpath::ConstPtr& message)
{
  ROS_INFO_STREAM("ADDING TOOLPATH " << message->id);

  bool create = true;
  ToolpathMarkerPtr marker;
  M_IDToToolpathMarker::iterator it = markers_.find(message->id);
  if (it != markers_.end())
  {
    marker = it->second;
    create = false;
  }

  if (create)
  {
    marker.reset(new ToolpathMarker(this, context_, scene_node_));
    if (marker)
    {
      markers_.insert(std::make_pair(message->id, marker));
    }
  }

  if (marker)
  {
    marker->setMessage(message);
    context_->queueRender();
  }
}

void GcodeDisplay::processDelete(const gcode_msgs::Toolpath::ConstPtr& message)
{
  ROS_INFO_STREAM("DELETING TOOLPATH" << message->id);
  deleteToolpath(message->id);
  context_->queueRender();
}

void GcodeDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  V_ToopathMessage local_queue;

  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    local_queue.swap(message_queue_);
  }

  if (!local_queue.empty())
  {
    V_ToopathMessage::iterator message_it = local_queue.begin();
    V_ToopathMessage::iterator message_end = local_queue.end();
    for (; message_it != message_end; ++message_it)
    {
      gcode_msgs::Toolpath::ConstPtr& marker = *message_it;
      processMessage(marker);
    }
  }
}

void GcodeDisplay::fixedFrameChanged() { clearMarkers(); }

void GcodeDisplay::reset()
{
  Display::reset();
  clearMarkers();
}

void GcodeDisplay::setTopic(const QString& topic, const QString& datatype)
{
  gcode_topic_property_->setString(topic);
}

}  // namespace gcode_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(gcode_rviz::GcodeDisplay, rviz::Display)
