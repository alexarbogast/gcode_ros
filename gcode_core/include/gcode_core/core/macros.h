#ifndef GCODE_CORE_MACROS_H
#define GCODE_CORE_MACROS_H

#include <memory>

#define GCODE_CORE_DECLARE_PTR(Name, Type)                                              \
    typedef std::shared_ptr<Type> Name##Ptr;                                            \
    typedef std::shared_ptr<const Type> Name##ConstPtr;                                 \
    typedef std::unique_ptr<Type> Name##UniquePtr;                                      \
    typedef std::unique_ptr<const Type> Name##ConstUniquePtr                            \
                
#define GCODE_CORE_CLASS_FORWARD(C)                                                     \
    class C;                                                                            \
    GCODE_CORE_DECLARE_PTR(C, C)                                                        \
                
#endif