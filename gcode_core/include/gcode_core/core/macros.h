#ifndef GCODE_CORE_MACROS_H
#define GCODE_CORE_MACROS_H

#include <memory>
#include <vector>

#define GCODE_CORE_DECLARE_PTR(Name, Type)                                              \
    typedef std::shared_ptr<Type> Name##Ptr;                                            \
    typedef std::shared_ptr<const Type> Name##ConstPtr;                                 \
    typedef std::unique_ptr<Type> Name##UniquePtr;                                      \
    typedef std::unique_ptr<const Type> Name##ConstUniquePtr                            \
                
#define GCODE_CORE_CLASS_FORWARD(C)                                                     \
    class C;                                                                            \
    GCODE_CORE_DECLARE_PTR(C, C)                                                        \

#define GCODE_CORE_CONTAINER_FORWARD(Class, Container)                                  \
    using iterator = typename std::vector<Class>::iterator;                             \
    using const_iterator = typename std::vector<Class>::const_iterator;                 \
    using reverse_iterator = typename std::vector<Class>::reverse_iterator;             \
    using const_reverse_iterator = typename std::vector<Class>::const_reverse_iterator; \
                                                                                        \
    iterator begin() { return Container.begin(); }                                      \
    const_iterator begin() const { return Container.begin(); }                          \
    iterator end() { return Container.end(); }                                          \
    const_iterator end() const { return Container.end(); }                              \
    reverse_iterator rbegin() { return Container.rbegin(); }                            \
    const_reverse_iterator rbegin() const { return Container.rbegin(); }                \
    reverse_iterator rend() { return Container.rend(); }                                \
    const_reverse_iterator rend() const { return Container.rend(); }                    \
    const_iterator cbegin() const { return Container.cbegin(); }                        \
    const_iterator cend() const { return Container.cend(); }                            \
    const_reverse_iterator crbegin() const { return Container.crbegin(); }              \
    const_reverse_iterator crend() const { return Container.crend(); }                  \
                                                                                        \
    bool empty() const { return Container.empty(); }                                    \
    size_t size() const { return Container.size(); }                                    \


#endif // GCODE_CORE_MACROS_H