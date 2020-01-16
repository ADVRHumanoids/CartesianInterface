#ifndef CONTEXT_H
#define CONTEXT_H

#include <memory>
#include <stdexcept>
#include <cartesian_interface/Macro.h>

namespace XBot { namespace Cartesian {

class ContextImpl;


class Context
{

public:

    CARTESIO_DECLARE_SMART_PTR(Context)

    Context();

    double getControlPeriod() const;

    ~Context();

    static Ptr MakeContext(double control_period);

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
