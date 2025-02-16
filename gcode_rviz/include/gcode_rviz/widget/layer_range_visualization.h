#ifndef LAYER_RANGE_VISUALIZATION_H
#define LAYER_RANGE_VISUALIZATION_H

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gcode_core/core/macros.h>
#include <gcode_msgs/Toolpath.h>

namespace gcode_rviz
{
// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
static const double DEFAULT_LINE_WIDTH = 5.0;

enum class ColorMethod
{
  ByTool,
  RandomByLayer,
  UniformLayers,
};

enum class DisplayStyle
{
  Lines,
  Cylinders
};

GCODE_CORE_CLASS_FORWARD(LayerRangeVisualization);
class LayerRangeVisualization
{
public:
  explicit LayerRangeVisualization(
      const std::string& marker_topic = RVIZ_MARKER_TOPIC);

  void update(int lower, int upper);
  void update(const std::set<std::size_t>& layers);

  void setLayers(const std::vector<gcode_msgs::Toolpath::Ptr>& layers);
  void setLayers(std::vector<gcode_msgs::Toolpath::Ptr>&& layers);
  inline std::size_t nLayers() const { return layers_.size(); }

protected:
  void publishLayer(int layer);
  void removeLayer(int layer);

  std::vector<gcode_msgs::Toolpath::Ptr> layers_;
  std::set<std::size_t> active_layers_;

  // ROS Visualization
  ros::NodeHandle nh_;
  ros::Publisher markers_pub_;
};

}  // namespace gcode_rviz

#endif  // LAYER_RANGE_VISUALIZATION_H
