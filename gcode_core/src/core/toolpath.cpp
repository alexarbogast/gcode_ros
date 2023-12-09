#include "gcode_core/core/toolpath.h"

namespace gcode_core
{

std::vector<ToolpathPtr> Toolpath::layers()
{
  // Find layer iterators
  std::vector<Toolpath::iterator> its;
  Toolpath::iterator ptr = this->begin();

  double current_z = -std::numeric_limits<double>::infinity();
  for (; ptr < this->end(); ptr++)
  {
    auto cmd = *ptr;
    if (cmd->getCommandType() == MoveCommandType::Extrusion &&
        cmd->translation().z() > current_z)
    {
      its.push_back(ptr);
      current_z = cmd->translation().z();
    }
  }

  its[0] = this->begin();
  its.push_back(this->end());

  std::vector<ToolpathPtr> layers; 
  for (std::size_t i = 0; i < its.size() - 1; i++)
  {
    ToolpathPtr toolpath = std::make_shared<Toolpath>();
    for (auto cit = its[i]; cit < its[i + 1]; ++cit)
    {
      toolpath->push_back(*cit);  
    }
    layers.push_back(toolpath);
  }
  return layers;
}

}  // namespace gcode_core
