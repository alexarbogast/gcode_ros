#ifndef TOOLPATH_H
#define TOOLPATH_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gcode_core/core/move_command.h>
#include <gcode_core/core/macros.h>

namespace gcode_core
{
GCODE_CORE_CLASS_FORWARD(Toolpath);
class Toolpath
{
public:
  Toolpath() = default;

  std::string to_string() const
  {
    std::stringstream ss;
    for (auto& cmd : commands_)
    {
      ss << *cmd;
    }
    return ss.str();
  }

  // clang-format off
  MoveCommand& back() { return *commands_.back(); }
  const MoveCommand& back() const { return *commands_.back(); }
  MoveCommand& front() { return *commands_.front(); }
  const MoveCommand& front() const { return *commands_.front(); }

  void push_back(std::shared_ptr<MoveCommand>&& x) { commands_.push_back(x); }
  void push_back(std::shared_ptr<MoveCommand>& x) { commands_.push_back(x); }

  MoveCommand& operator[](size_t pos) { return *commands_[pos]; }
  const MoveCommand& operator[](size_t pos) const { return *commands_[pos]; }
  // clang-format on

  GCODE_CORE_CONTAINER_FORWARD(std::shared_ptr<MoveCommand>, commands_)

  /**
   * \brief Breaks a toolpath into layers at unique z-values
   *
   * Layers are returned as a vector of `Toolpath`s.
   * Extrusion commands are assumed to be in strictly increasing order
   *
   * \return std::vector<std::shared_ptr<Toolpath>>
   */
  std::vector<ToolpathPtr> layers();

private:
  // std::vector<std::shared_ptr<Layer>> layers_;
  std::vector<std::shared_ptr<MoveCommand>> commands_;
};

inline std::ostream& operator<<(std::ostream& os, const Toolpath& toolpath)
{
  return os << toolpath.to_string();
}

}  // namespace gcode_core

#endif  // TOOLPATH_H
