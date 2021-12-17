#ifndef GCODE_BASE_H
#define GCODE_BASE_H

#include <vector>

#include "gcode_core/core/command.h"
#include "gcode_core/core/macros.h"

namespace gcode_core
{
GCODE_CORE_CLASS_FORWARD(GcodeBase);

class GcodeBase
{
public:
    GcodeBase() = default;
    virtual ~GcodeBase() = default;

    virtual void parse(std::stringstream& ss) = 0;
    virtual void print() const;

    friend std::stringstream& operator>>(std::stringstream& ss, GcodeBase& cmd);

    void push_back(const Command& x) { container_.push_back(x); }
    void push_back(const Command&& x) { container_.push_back(x); }

    using iterator = typename std::vector<Command>::iterator;
    using const_iterator = typename std::vector<Command>::const_iterator;
    using reverse_iterator = typename std::vector<Command>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<Command>::const_reverse_iterator;

    ///////////////
    // Iterators //
    ///////////////
    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;
    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    reverse_iterator rend();
    const_reverse_iterator rend() const;
    const_iterator cbegin() const;
    const_iterator cend() const;
    const_reverse_iterator crbegin() const;
    const_reverse_iterator crend() const;

protected:
    std::vector<Command> container_;
};

} // namespace gcode_core

#endif // GCODE_BASE_H