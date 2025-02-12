#include <gcode_rviz/widget/layer_range_visualization.h>

#include <tf2_eigen/tf2_eigen.h>
#include <unordered_map>

namespace gcode_rviz
{
const std_msgs::ColorRGBA CARBON = create_color_rgba(51, 51, 51, 255);
const std_msgs::ColorRGBA FEROCIOUS_FOX = create_color_rgba(223, 91, 30, 255);
const std_msgs::ColorRGBA GREEN_BLUE = create_color_rgba(58, 183, 149, 255);
const std_msgs::ColorRGBA BROCADE = create_color_rgba(139, 133, 193, 255);
const std_msgs::ColorRGBA MELTED_BUTTER = create_color_rgba(255, 207, 86, 255);
const std_msgs::ColorRGBA GUNMETAL = create_color_rgba(36, 47, 64, 255);
const std_msgs::ColorRGBA SATIN_GOLD = create_color_rgba(204, 164, 59, 255);
const std_msgs::ColorRGBA TOMATO = create_color_rgba(249, 87, 56, 255);
const std_msgs::ColorRGBA KELLY_GREEN = create_color_rgba(67, 185, 41, 255);

static const int N_COLORS = 8;
static std::unordered_map<int, std_msgs::ColorRGBA>
    ToolColor({ { 0, FEROCIOUS_FOX },
                { 1, GREEN_BLUE },
                { 2, BROCADE },
                { 3, MELTED_BUTTER },
                { 4, GUNMETAL },
                { 5, SATIN_GOLD },
                { 6, TOMATO },
                { 7, KELLY_GREEN } });

std_msgs::ColorRGBA create_color_rgba(uint8_t r, uint8_t g, uint8_t b,
                                      uint8_t a)
{
  std_msgs::ColorRGBA color;

  color.r = (float)r / 255;
  color.g = (float)g / 255;
  color.b = (float)b / 255;
  color.a = (float)a / 255;

  return color;
}

LayerRangeVisualization::LayerRangeVisualization(
    const std::string& base_frame, const std::string& marker_topic)
{
  // initRvizMarkers();
  rvt_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(base_frame,
                                                              marker_topic);
  rvt_->loadMarkerPub();
  rvt_->enableBatchPublishing();
  rvt_->enableFrameLocking();

  color_ = FEROCIOUS_FOX;
}

void LayerRangeVisualization::update(int lower, int upper, bool reset)
{
  if (layers_.size() == 0)
    return;

  rvt_->deleteAllMarkers();
  rvt_->resetMarkerCounts();
  for (int i = lower; i <= upper; ++i)
  {
    publishLayer(i);
  }
  rvt_->trigger();
}

void LayerRangeVisualization::setLayers(
    const std::vector<gcode_core::ToolpathPtr>& layers)
{
  layers_ = layers;
  resetRandomColorMap();
}

void LayerRangeVisualization::setLayers(
    std::vector<gcode_core::ToolpathPtr>&& layers)
{
  layers_ = std::move(layers);
  resetRandomColorMap();
}

void LayerRangeVisualization::resetRandomColorMap()
{
  random_layer_colors_.clear();
  random_layer_colors_.resize(nLayers());
  for (auto& color : random_layer_colors_)
  {
    color =
        ToolColor[rviz_visual_tools::RvizVisualTools::iRand(0, N_COLORS - 1)];
  }
}

/* =============== Rviz Visual Tools ============== */
void LayerRangeVisualization::publishLayer(int layer)
{
  // ============ layer color ==============
  std_msgs::ColorRGBA layer_color = CARBON;
  switch (color_method_)
  {
    case ColorMethod::RandomByLayer:
      layer_color = random_layer_colors_[layer];
      break;
    case ColorMethod::UniformLayers:
      layer_color = color_;
      break;
    default:
      break;
  }

  // ============ display style ==============
  EigenSTL::vector_Vector3d path;
  std::vector<std_msgs::ColorRGBA> colors;
  for (auto& cmd : *layers_[layer])
  {
    std_msgs::ColorRGBA cmd_color = layer_color;
    path.emplace_back(cmd->translation() / 1000);
    getCommandColor(*cmd, cmd_color);
    colors.emplace_back(cmd_color);
  }

  switch (display_style_)
  {
    case DisplayStyle::Cylinders:
      rvt_->publishPath(path, colors, line_width_);
      break;
    case DisplayStyle::Lines:
    default:
      break;
  }
}

void LayerRangeVisualization::getCommandColor(
    const gcode_core::MoveCommand& cmd, std_msgs::ColorRGBA& color) const
{
  if (cmd.getCommandType() == gcode_core::MoveCommandType::Travel)
  {
    color = CARBON;
    if (hide_travel_)
      color.a = 0.0;
    return;
  }

  switch (color_method_)
  {
    case ColorMethod::ByTool:
      color = ToolColor[cmd.getTool() % N_COLORS];
      break;
    default:
      break;
  }
}

}  // namespace gcode_rviz
