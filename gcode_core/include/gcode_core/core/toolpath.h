#ifndef TOOLPATH_H
#define TOOLPATH_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gcode_core/core/move_command.h>
#include <gcode_core/core/macros.h>

namespace gcode_core
{

class Bead
{
public:
    Bead() = default;

    std::string to_string() const
    {
        std::stringstream ss;
        ss << "Bead: " << "\n";

        for (auto& cmd : segments_)
            ss << "\t" << *cmd;

        return ss.str();
    }

    void push_back(std::shared_ptr<MoveCommand>&& x) { segments_.push_back(x); }
    void push_back(std::shared_ptr<MoveCommand>& x) { segments_.push_back(x); }

    MoveCommand& back() { return *segments_.back(); }
    const MoveCommand& back() const { return *segments_.back(); }
    MoveCommand& front() { return *segments_.front(); }
    const MoveCommand& front() const { return *segments_.front(); }

    MoveCommand& operator[] (size_t pos) { return *segments_[pos]; }
    const MoveCommand& operator[] (size_t pos) const { return *segments_[pos]; }

    GCODE_CORE_CONTAINER_FORWARD(std::shared_ptr<MoveCommand>, segments_)
private:
    std::vector<std::shared_ptr<MoveCommand>> segments_;
};

inline std::ostream& operator<<(std::ostream& os, const Bead& bead)
{
    return os << bead.to_string();
}

class Layer
{
public:
    Layer() = default;

    std::string to_string() const
    {
        std::stringstream ss;

        for (int i = 0; i < beads_.size(); i++)
        {
            ss << "Bead: " << i << "\n";
            for (auto& cmd : *beads_[i])
                ss << "\t" << *cmd;
        }
        return ss.str();
    }

    void push_back(std::shared_ptr<Bead>&& x) { beads_.push_back(x); }
    void push_back(std::shared_ptr<Bead>& x) { beads_.push_back(x); }

    Bead& back() { return *beads_.back(); }
    const Bead& back() const { return *beads_.back(); }
    Bead& front() { return *beads_.front(); }
    const Bead& front() const { return *beads_.front(); }

    Bead& operator[] (size_t pos) { return *beads_[pos]; }
    const Bead& operator[] (size_t pos) const { return *beads_[pos]; }

    GCODE_CORE_CONTAINER_FORWARD(std::shared_ptr<Bead>, beads_)
private:
    std::vector<std::shared_ptr<Bead>> beads_;
};

inline std::ostream& operator<<(std::ostream& os, const Layer& layer)
{
    return os << layer.to_string();
}

class Toolpath
{
public:
    Toolpath() = default;

    std::string to_string() const
    {
        std::stringstream ss;

        for (int i = 0; i < layers_.size(); i++)
        {
            ss << "Layer: " << i << "\n";
            for (int j = 0; j < layers_[i]->size(); j++)
            {
                ss << "\tBead: " << j << "\n";
                for (auto& cmd : (*layers_[i])[j])
                    ss << "\t\t" << *cmd;
            } 
        }
        return ss.str();
    }

    Layer& back() { return *layers_.back(); }                                         
    const Layer& back() const { return *layers_.back(); }                             
    Layer& front() { return *layers_.front(); }                                       
    const Layer& front() const { return *layers_.front(); }

    void push_back(std::shared_ptr<Layer>&& x) { layers_.push_back(x); }
    void push_back(std::shared_ptr<Layer>& x) { layers_.push_back(x); }

    Layer& operator[] (size_t pos) { return *layers_[pos]; }
    const Layer& operator[] (size_t pos) const { return *layers_[pos]; }

    GCODE_CORE_CONTAINER_FORWARD(std::shared_ptr<Layer>, layers_)
private:
    std::vector<std::shared_ptr<Layer>> layers_;
};

inline std::ostream& operator<<(std::ostream& os, const Toolpath& toolpath)
{
    return os << toolpath.to_string();
}

} // namespace gcode_core

#endif // TOOLPATH_H