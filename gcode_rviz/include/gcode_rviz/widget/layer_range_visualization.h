#ifndef LAYER_RANGE_VISUALIZATION_H
#define LAYER_RANGE_VISUALIZATION_H

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Pose.h>

#include <gcode_core/core/toolpath.h>

namespace gcode_rviz
{
// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
static const double DEFAULT_LINE_WIDTH = 5.0;

std_msgs::ColorRGBA create_color_rgba(uint8_t r, uint8_t g, uint8_t b,
                                      uint8_t a);

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

  void setLayers(const std::vector<gcode_core::ToolpathPtr>& layers);
  void setLayers(std::vector<gcode_core::ToolpathPtr>&& layers);
  inline std::size_t nLayers() const { return layers_.size(); }

  inline void setLineWidth(double line_width) { line_width_ = line_width; }
  inline void setHideTravel(bool hide_travel) { hide_travel_ = hide_travel; }
  inline void setColor(const std_msgs::ColorRGBA& color) { color_ = color; }
  inline void setColorMethod(const ColorMethod& method)
  {
    color_method_ = method;
  }
  inline void setBaseFrame(const std::string& base_frame)
  {
    rvt_->setBaseFrame(base_frame);
  }
  inline void setDisplayStyle(const DisplayStyle& style)
  {
    display_style_ = style;
  }

  void resetRandomColorMap();

  void publishLayer(int layer);
  void getCommandColor(const gcode_core::MoveCommand& cmd,
                       std_msgs::ColorRGBA& color) const;

protected:
  rviz_visual_tools::RvizVisualToolsPtr rvt_;
  std::vector<gcode_core::ToolpathPtr> layers_;

  std::vector<std_msgs::ColorRGBA> random_layer_colors_;

  // Display options
  double line_width_ = 0.001;
  bool hide_travel_ = true;
  std_msgs::ColorRGBA color_;
  ColorMethod color_method_ = ColorMethod::ByTool;
  DisplayStyle display_style_ = DisplayStyle::Cylinders;
};

}  // namespace gcode_rviz

#endif  // LAYER_RANGE_VISUALIZATION_H
