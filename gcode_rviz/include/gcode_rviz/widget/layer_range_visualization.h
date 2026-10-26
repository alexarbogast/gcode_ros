#ifndef LAYER_RANGE_VISUALIZATION_H
#define LAYER_RANGE_VISUALIZATION_H

// #include <ros/callback_queue.h>
#include <rclcpp/rclcpp.hpp>

#include <random>

#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gcode_core/core/toolpath.h>

namespace gcode_rviz
{
// Default constants
inline constexpr const char* RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
inline constexpr double DEFAULT_LINE_WIDTH = 5.0;

std_msgs::msg::ColorRGBA create_color_rgba(uint8_t r, uint8_t g, uint8_t b,
                                           uint8_t a);

geometry_msgs::msg::Pose create_identity_pose();

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
      const std::string& base_frame,
      const std::string& marker_topic = RVIZ_MARKER_TOPIC);

  void update(int lower, int upper, bool reset = true);
  void update(const std::set<std::size_t>& layers);

  void setLayers(const std::vector<gcode_core::ToolpathPtr>& layers);
  void setLayers(std::vector<gcode_core::ToolpathPtr>&& layers);
  inline std::size_t nLayers() const { return layers_.size(); }

  inline void setLineWidth(double line_width) { line_width_ = line_width; }
  inline void setHideTravel(bool hide_travel) { hide_travel_ = hide_travel; }
  inline void setColor(const std_msgs::msg::ColorRGBA& color)
  {
    color_ = color;
  }
  inline void setColorMethod(const ColorMethod& method)
  {
    color_method_ = method;
  }
  inline void setBaseFrame(const std::string& base_frame)
  {
    base_frame_ = base_frame;
  }
  inline void setDisplayStyle(const DisplayStyle& style)
  {
    display_style_ = style;
  }

protected:
  void resetRandomColorMap();
  void getCommandColor(const gcode_core::MoveCommand& cmd,
                       std_msgs::msg::ColorRGBA& color) const;

  void initializeMarkers();
  void reset();
  void trigger();

  void publishLayer(int layer);
  bool publishPath(const gcode_core::ToolpathPtr& path,
                   const std_msgs::msg::ColorRGBA& colors,
                   const std::string& ns);
  bool publishCylinder(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                       const std_msgs::msg::ColorRGBA& color,
                       const std::string& ns);

  void removeLayer(int layer);

  static std::mt19937 mt_random_engine_;

  std::vector<gcode_core::ToolpathPtr> layers_;
  std::set<std::size_t> active_layers_;

  // Display options
  double line_width_ = 0.001;
  bool hide_travel_ = true;
  std::string base_frame_;
  std_msgs::msg::ColorRGBA color_;
  std::vector<std_msgs::msg::ColorRGBA> random_layer_colors_;
  ColorMethod color_method_ = ColorMethod::ByTool;
  DisplayStyle display_style_ = DisplayStyle::Cylinders;

  // ROS Visualization
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  rclcpp::Duration marker_lifetime_{ 0, 0 };

  visualization_msgs::msg::MarkerArray marker_array_;
  visualization_msgs::msg::Marker cylinder_template_;
  visualization_msgs::msg::Marker reset_template_;
};

}  // namespace gcode_rviz

#endif  // LAYER_RANGE_VISUALIZATION_H
