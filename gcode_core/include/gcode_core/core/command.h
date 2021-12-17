#ifndef COMMAND_H
#define COMMAND_H

#include <memory>
#include <vector>
#include <sstream>

namespace gcode_core
{
namespace detail_command
{
struct CommandInnerBase
{
    CommandInnerBase() = default;
    virtual ~CommandInnerBase() = default;
    CommandInnerBase(const CommandInnerBase&) = delete;
    CommandInnerBase& operator=(const CommandInnerBase&) = delete;
    CommandInnerBase(CommandInnerBase&&) = delete;
    CommandInnerBase& operator=(CommandInnerBase&&) = delete;

    virtual std::unique_ptr<CommandInnerBase> clone() const = 0;

    // User-defined methods
    virtual void parse(std::stringstream& args) = 0;
    virtual void print() const = 0;
};

template <typename T>
struct CommandInner final : CommandInnerBase
{
    CommandInner() = default;
    ~CommandInner() override = default;
    CommandInner(const CommandInner&) = delete;
    CommandInner(CommandInner&&) = delete;
    CommandInner& operator=(const CommandInner&) = delete;
    CommandInner& operator=(CommandInner&&) = delete;

    explicit CommandInner(T command) : command_(std::move(command)) {}
    explicit CommandInner(T&& command) : command_(std::move(command)) {}

    std::unique_ptr<CommandInnerBase> clone() const final { return std::make_unique<CommandInner>(command_); }

    // User defined methods
    void parse(std::stringstream& args) final { command_.parse(args); }
    void print() const final { command_.print(); }

private:
    T command_;
};
} // namespace detail_command

class Command
{
    // aliases to simplify some SFINAE code below
    template <typename T>
    using uncvref_t = std::remove_cv_t<typename std::remove_reference<T>::type>;

    template <typename T>
    using generic_ctor_enabler = std::enable_if_t<!std::is_same<Command, uncvref_t<T>>::value, int>;

public:
    template <typename T, generic_ctor_enabler<T> = 0>
    Command(T&& command)
        : command_(std::make_unique<detail_command::CommandInner<uncvref_t<T>>>(command))
    {}   

    ~Command() = default;

    // Copy constructor
    Command(const Command& other) { command_ = other.command_->clone(); }
    // Move constructor
    Command(Command&& other) noexcept { command_.swap(other.command_); }
    
    // Copy Asignment
    Command& operator=(const Command& other)
    {
        (*this) = Command(other);
        return (*this);
    }
    // Move assignment
    Command& operator=(Command&& other) noexcept
    {
        command_.swap(other.command_);
        return (*this);
    }

    template <typename T, generic_ctor_enabler<T> = 0>
    Command& operator=(T&& other)
    {
      (*this) = Command(std::forward<T>(other));
      return (*this);
    }

    void parse(std::stringstream& args) const { command_->parse(args); }
    void print() const { command_->print(); }
private:
    Command()  // NOLINT
    : command_(nullptr)
    { }

    std::unique_ptr<detail_command::CommandInnerBase> command_;
};

} //namespace gcode_core

#endif // COMMAND_H