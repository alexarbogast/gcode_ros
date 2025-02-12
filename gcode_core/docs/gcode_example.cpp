#include <iostream>
#include <ros/package.h>

#include <gcode_core/core/interpreter.h>

using namespace gcode_core;

int main()
{
  // find example marlin gcode filepath
  std::string marlin_filepath = ros::package::getPath("gcode_core") +
                         "/resources/simple_cube_marlin.gcode";

  std::string reprap_filepath = ros::package::getPath("gcode_core") +
                         "/resources/multi_tool_square_reprap.gcode";

  // Instantiate gcode object and parse file
  Toolpath marlin_gcode;
  Toolpath reprap_gcode;

  //Marlin::ParseGcode(filepath, gcode);

  std::shared_ptr<GcodeInterpreter> interpreter = std::make_shared<GcodeInterpreter>();
  interpreter->parseGcode(marlin_filepath, marlin_gcode);
  interpreter->parseGcode(reprap_filepath, reprap_gcode);

  std::vector<ToolpathPtr> marlin_layers = marlin_gcode.layers();
  std::vector<ToolpathPtr> reprap_layers = reprap_gcode.layers();
  
  std::cout << marlin_layers.size() << std::endl;
  std::cout << reprap_layers.size() << std::endl;

  //std::cout << "marlin layer0" << std::endl;
  //for (auto& cmd : *marlin_layers[0])
  //{
  //  std::cout << *cmd;
  //}
//
//  //std::cout << "reprap layer0" << std::endl;
  //for (auto& cmd : *reprap_layers[0])
  //{
  //  std::cout << *cmd;
  //}
  std::cout << *marlin_layers[0] << std::endl;
  return 0;
}
