#ifndef CONTEXT_H
#define CONTEXT_H

#include <memory>
#include <stdexcept>

namespace XBot { namespace Cartesian {

struct ContextImpl;

class Context
{

public:

    Context();
    explicit Context(std::shared_ptr<ContextImpl> ctx);

    double getControlPeriod() const;

    ~Context();

private:

    static std::weak_ptr<ContextImpl> _weak_impl;

    std::shared_ptr<ContextImpl> _impl;

};

struct ContextEmpty : public std::exception
{
    const char * what() const noexcept override;
};

struct ContextInvalid : public std::exception
{
    ContextInvalid(std::string reason);
    const char * what() const noexcept override;

private:

    std::string _reason;
};

struct ContextRedefinition : public std::exception
{
    const char * what() const noexcept override;
};


} }


#endif // CONTEXT_H
