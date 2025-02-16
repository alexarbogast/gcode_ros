#include <gcode_rviz/widget/layer_range_visualization.h>
#include <gcode_msgs/Toolpath.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include <numeric>

namespace gcode_rviz
{

geometry_msgs::Pose create_identity_pose()
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  return pose;
}

LayerRangeVisualization::LayerRangeVisualization(
    const std::string& marker_topic)
{
  nh_ = ros::NodeHandle("~");
  markers_pub_ = nh_.advertise<gcode_msgs::Toolpath>(marker_topic, 10, true);
  ROS_DEBUG_STREAM("Publishing gcode visualization marker on topic"
                   << marker_topic);
}

void LayerRangeVisualization::update(int lower, int upper)
{
  if (layers_.size() == 0)
    return;

  std::vector<std::size_t> idx(upper - lower + 1);
  std::iota(idx.begin(), idx.end(), lower);
  std::set<std::size_t> new_layers(idx.begin(), idx.end());
  update(new_layers);
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

  active_layers_ = layers;
}

void LayerRangeVisualization::setLayers(
    const std::vector<gcode_msgs::Toolpath::Ptr>& layers)
{
  layers_ = layers;
}

void LayerRangeVisualization::setLayers(
    std::vector<gcode_msgs::Toolpath::Ptr>&& layers)
{
  layers_ = std::move(layers);
}

void LayerRangeVisualization::publishLayer(int layer)
{
  markers_pub_.publish(layers_[layer]);
}

void LayerRangeVisualization::removeLayer(int layer)
{
  gcode_msgs::Toolpath delete_layer_msg;
  delete_layer_msg.action = gcode_msgs::Toolpath::DELETE;
  delete_layer_msg.id = layer;
  markers_pub_.publish(delete_layer_msg);
}

}  // namespace gcode_rviz
