#include <gcode_core/core/move_command.h>
#include <gcode_rviz/widget/layer_range_visualization.h>

#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

#include <unordered_map>

namespace gcode_rviz
{
// clang-format off
const std_msgs::msg::ColorRGBA CARBON = create_color_rgba(51, 51, 51, 255);
const std_msgs::msg::ColorRGBA FEROCIOUS_FOX = create_color_rgba(223, 91, 30, 255);
const std_msgs::msg::ColorRGBA GREEN_BLUE = create_color_rgba(58, 183, 149, 255);
const std_msgs::msg::ColorRGBA BROCADE = create_color_rgba(139, 133, 193, 255);
const std_msgs::msg::ColorRGBA MELTED_BUTTER = create_color_rgba(255, 207, 86, 255);
const std_msgs::msg::ColorRGBA GUNMETAL = create_color_rgba(36, 47, 64, 255);
const std_msgs::msg::ColorRGBA SATIN_GOLD = create_color_rgba(204, 164, 59, 255);
const std_msgs::msg::ColorRGBA TOMATO = create_color_rgba(249, 87, 56, 255);
const std_msgs::msg::ColorRGBA KELLY_GREEN = create_color_rgba(67, 185, 41, 255);
// clang-format on

static const int N_COLORS = 8;
static std::unordered_map<int, std_msgs::msg::ColorRGBA>
    ToolColor({ { 0, FEROCIOUS_FOX },
                { 1, GREEN_BLUE },
                { 2, BROCADE },
                { 3, MELTED_BUTTER },
                { 4, GUNMETAL },
                { 5, SATIN_GOLD },
                { 6, TOMATO },
                { 7, KELLY_GREEN } });

std::mt19937 LayerRangeVisualization::mt_random_engine_(1415926);

std_msgs::msg::ColorRGBA create_color_rgba(uint8_t r, uint8_t g, uint8_t b,
                                           uint8_t a)
{
  std_msgs::msg::ColorRGBA color;

  color.r = (float)r / 255;
  color.g = (float)g / 255;
  color.b = (float)b / 255;
  color.a = (float)a / 255;

  return color;
}

geometry_msgs::msg::Pose create_identity_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  return pose;
}

LayerRangeVisualization::LayerRangeVisualization(
    const std::string& base_frame, const std::string& marker_topic)
  : base_frame_(base_frame), color_(FEROCIOUS_FOX)
{
  if (!node_)
  {
    node_ = std::make_shared<rclcpp::Node>("layer_range_visualization");
  }

  markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, rclcpp::QoS(10).transient_local());

  RCLCPP_DEBUG(node_->get_logger(),
               "Publishing gcode visualization marker on topic %s",
               marker_topic.c_str());

  initializeMarkers();
}

void LayerRangeVisualization::initializeMarkers()
{
  // reset marker
  reset_template_.header.frame_id = base_frame_;
  reset_template_.header.stamp = node_->get_clock()->now();
  reset_template_.action = visualization_msgs::msg::Marker::DELETEALL;
  reset_template_.pose = create_identity_pose();

  // cylinder marker
  cylinder_template_.action = visualization_msgs::msg::Marker::ADD;
  cylinder_template_.type = visualization_msgs::msg::Marker::CYLINDER;
  cylinder_template_.lifetime = marker_lifetime_;
  cylinder_template_.pose = create_identity_pose();
}

void LayerRangeVisualization::update(int lower, int upper, bool reset)
{
  if (layers_.size() == 0)
    return;

  if (reset)
    this->reset();

  std::vector<std::size_t> idx(upper - lower + 1);
  std::iota(idx.begin(), idx.end(), lower);
  std::set<std::size_t> new_layers(idx.begin(), idx.end());
  update(new_layers);
}

void LayerRangeVisualization::reset()
{
  visualization_msgs::msg::Marker reset_marker = reset_template_;
  marker_array_.markers.push_back(reset_marker);
  trigger();
  active_layers_.clear();
}

void LayerRangeVisualization::update(const std::set<std::size_t>& layers)
{
  std::set<std::size_t> del, add;

  // find the active elements to delete
  std::set_difference(active_layers_.begin(), active_layers_.end(),
                      layers.begin(), layers.end(),
                      std::inserter(del, del.begin()));

  // find layers to add to active elements
  std::set_difference(layers.begin(), layers.end(), active_layers_.begin(),
                      active_layers_.end(), std::inserter(add, add.begin()));

  for (std::size_t layer : del)
  {
    removeLayer(layer);
  }

  for (std::size_t layer : add)
  {
    publishLayer(layer);
  }

  trigger();
  active_layers_ = layers;
}

void LayerRangeVisualization::setLayers(
    const std::vector<gcode_core::ToolpathPtr>& layers)
{
  reset();
  layers_ = layers;
  resetRandomColorMap();
}

void LayerRangeVisualization::setLayers(
    std::vector<gcode_core::ToolpathPtr>&& layers)
{
  reset();
  layers_ = std::move(layers);
  resetRandomColorMap();
}

void LayerRangeVisualization::resetRandomColorMap()
{
  random_layer_colors_.clear();
  random_layer_colors_.resize(nLayers());
  for (auto& color : random_layer_colors_)
  {
    color = ToolColor[std::uniform_int_distribution<int>(0, N_COLORS - 1)(
        mt_random_engine_)];
  }
}

void LayerRangeVisualization::publishLayer(int layer)
{
  // ============ layer color ==============
  std_msgs::msg::ColorRGBA layer_color = CARBON;
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

  // reset id counter for each layer
  cylinder_template_.id = 0;

  switch (display_style_)
  {
    case DisplayStyle::Cylinders:
      publishPath(layers_[layer], layer_color, "layer" + std::to_string(layer));
      break;
    case DisplayStyle::Lines:
    default:
      break;
  }
}

bool LayerRangeVisualization::publishPath(const gcode_core::ToolpathPtr& path,
                                          const std_msgs::msg::ColorRGBA& color,
                                          const std::string& ns)
{
  if (path->size() < 2)
  {
    RCLCPP_WARN(node_->get_logger(),
                "Skipping path because it contains %lu points", path->size());
    return false;
  }

  for (std::size_t i = 1; i < path->size(); ++i)
  {
    auto& cmd_start = (*path)[i - 1];
    auto& cmd_end = (*path)[i];

    if (hide_travel_ &&
        cmd_end.getCommandType() == gcode_core::MoveCommandType::Travel)
    {
      continue;
    }

    std_msgs::msg::ColorRGBA cmd_color = color;
    getCommandColor(cmd_end, cmd_color);

    publishCylinder(cmd_start.translation() / 1000,
                    cmd_end.translation() / 1000, cmd_color, ns);
  }

  return true;
}

bool LayerRangeVisualization::publishCylinder(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const std_msgs::msg::ColorRGBA& color, const std::string& ns)
{
  Eigen::Vector3d axis_vector = end - start;
  double dist = axis_vector.norm();

  if (std::abs(dist) < std::numeric_limits<double>::epsilon())
  {
    RCLCPP_WARN(node_->get_logger(), "Cylinder length smaller than epsilon");

    return false;
  }

  axis_vector.normalize();

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = up_vector.cross(axis_vector);
  right_axis_vector.normalize();

  double theta = acos(up_vector.dot(axis_vector));

  Eigen::Quaterniond q;
  q = Eigen::AngleAxis<double>(theta, right_axis_vector);
  q.normalize();

  Eigen::Isometry3d pose;
  pose = q;
  pose.translation() = 0.5 * (start + end);

  visualization_msgs::msg::Marker cylinder = cylinder_template_;
  cylinder.header.stamp = node_->get_clock()->now();
  cylinder.header.frame_id = base_frame_;
  cylinder.ns = ns;
  cylinder_template_.id++;

  tf2::convert(pose, cylinder.pose);
  cylinder.scale.x = line_width_;
  cylinder.scale.y = line_width_;
  cylinder.scale.z = dist;

  cylinder.color = color;
  marker_array_.markers.push_back(cylinder);
  return true;
}

void LayerRangeVisualization::removeLayer(int layer)
{
  std::size_t max_id = layers_[layer]->size();
  for (std::size_t i = 0; i < max_id; ++i)
  {
    visualization_msgs::msg::Marker reset_marker = reset_template_;
    reset_marker.id = i;
    reset_marker.ns = "layer" + std::to_string(layer);
    reset_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array_.markers.push_back(reset_marker);
  }
}

void LayerRangeVisualization::trigger()
{
  markers_pub_->publish(marker_array_);
  marker_array_.markers.clear();
}

void LayerRangeVisualization::getCommandColor(
    const gcode_core::MoveCommand& cmd, std_msgs::msg::ColorRGBA& color) const
{
  if (cmd.getCommandType() == gcode_core::MoveCommandType::Travel)
  {
    color = CARBON;
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
