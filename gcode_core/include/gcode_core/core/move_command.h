#ifndef MOVE_COMMAND_H
#define MOVE_COMMAND_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gcode_core
{
enum class MoveCommandType
{
    None = 0,
    Travel, Extrusion
};

class MoveCommand
{
public:
    MoveCommand() = default;

    MoveCommand(const MoveCommand& other)
    {
        waypoint_ = other.getWaypoint();
        type_ = other.getCommandType();
    }

    std::string to_string() const
    {
        auto t = this->translation();
        std::stringstream ss;

        ss << "MoveCommand: {" << t.x() << ", " << t.y() << ", "<< t.z() << "} Type " << (int)type_ <<"\n";
        return ss.str(); 
    }

    void setCommandType(const MoveCommandType& type) { type_ = type; }
    const MoveCommandType& getCommandType() const { return type_; }

    void setWaypoint(Eigen::Isometry3d waypoint) { waypoint_ = std::move(waypoint); }
    Eigen::Isometry3d& getWaypoint() { return waypoint_; }
    const Eigen::Isometry3d& getWaypoint() const { return waypoint_; };

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

    friend bool operator==(const MoveCommand& cmd1, const MoveCommand& cmd2);
    friend bool operator!=(const MoveCommand& cmd1, const MoveCommand& cmd2);

private:
    Eigen::Isometry3d waypoint_{ Eigen::Isometry3d::Identity() };
    MoveCommandType type_ = MoveCommandType::None;
};

inline std::ostream& operator<<(std::ostream&os, const MoveCommand& cmd)
{
    return os << cmd.to_string();
}

inline bool operator==(const gcode_core::MoveCommand& cmd1, const gcode_core::MoveCommand& cmd2)
{
    return cmd1.getWaypoint().isApprox(cmd2.getWaypoint()) && 
           cmd1.getCommandType() == cmd2.getCommandType();
}

inline bool operator!=(const gcode_core::MoveCommand& cmd1, const gcode_core::MoveCommand& cmd2)
{
    return !(cmd1 == cmd2);
}

} //namespace gcode_core

#endif // MOVE_COMMAND_H