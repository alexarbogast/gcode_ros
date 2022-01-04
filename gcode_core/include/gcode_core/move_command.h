#ifndef MOVE_COMMAND_H
#define MOVE_COMMAND_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gcode_core/core/command.h"

namespace gcode_core
{
class MoveCommand
{
public:
    MoveCommand() = default;

    virtual void parse(std::stringstream& args) = 0;
    virtual void print() const = 0;

    void setWaypoint(Eigen::Isometry3d waypoint) { waypoint_ = std::move(waypoint); }
    Eigen::Isometry3d& getWaypoint() { return waypoint_; }
    const Eigen::Isometry3d& getWaypoint() const { return waypoint_; };

    friend std::stringstream& operator>>(std::stringstream& ss, MoveCommand& cmd)
    {
        cmd.parse(ss);
        return ss;
    }

    using ConstLinearPart = Eigen::Isometry3d::ConstLinearPart;
    using LinearPart = Eigen::Isometry3d::LinearPart;
    using ConstTranslationPart = Eigen::Isometry3d::ConstTranslationPart;
    using TranslationPart = Eigen::Isometry3d::TranslationPart;

    inline ConstLinearPart linear() const { return waypoint_.linear(); }
    inline LinearPart linear() { return waypoint_.linear(); }
    inline ConstTranslationPart translation() const { return waypoint_.translation(); }
    inline TranslationPart translation() { return waypoint_.translation(); }

    inline MoveCommand& operator=(const Eigen::Isometry3d other)
    {
        waypoint_ = other;
        return *this;
    }

    // implicit eigen conversions
    inline operator const Eigen::Isometry3d&() const { return waypoint_; }
    inline operator Eigen::Isometry3d&() { return waypoint_; }

    COMMAND_CLASS_TYPE(MoveCommand)
protected:
    Eigen::Isometry3d waypoint_{ Eigen::Isometry3d::Identity() };
};

} //namespace gcode_core

#endif // MOVE_COMMAND_H