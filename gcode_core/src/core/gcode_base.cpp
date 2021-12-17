#include "gcode_core/core/gcode_base.h"

namespace gcode_core
{
void GcodeBase::print() const
{
    for (auto cmd : container_)
        cmd.print();
}

std::stringstream& operator>>(std::stringstream& ss, GcodeBase& cmd)
{
    cmd.parse(ss);
    return ss;
}

// Iterators 
GcodeBase::iterator GcodeBase::begin() { return container_.begin(); }
GcodeBase::const_iterator GcodeBase::begin() const { return container_.begin(); }
GcodeBase::iterator GcodeBase::end() { return container_.end(); }
GcodeBase::const_iterator GcodeBase::end() const { return container_.end(); }
GcodeBase::reverse_iterator GcodeBase::rbegin() { return container_.rbegin(); }
GcodeBase::const_reverse_iterator GcodeBase::rbegin() const { return container_.rbegin(); }
GcodeBase::reverse_iterator GcodeBase::rend() { return container_.rend(); }
GcodeBase::const_reverse_iterator GcodeBase::rend() const { return container_.rend(); }
GcodeBase::const_iterator GcodeBase::cbegin() const { return container_.cbegin(); }
GcodeBase::const_iterator GcodeBase::cend() const { return container_.cend(); }
GcodeBase::const_reverse_iterator GcodeBase::crbegin() const { return container_.crbegin(); }
GcodeBase::const_reverse_iterator GcodeBase::crend() const { return container_.crend(); }

} // namespace gcode_core