#ifndef DYNAMICLOADING_H
#define DYNAMICLOADING_H

#ifndef LOAD_OBJECT_H
#define LOAD_OBJECT_H

#include <cstdio>
#include <memory>
#include <string>
#include <dlfcn.h>

struct LibNotFound : public std::exception
{
    const char * what() const noexcept override
    {
        return dlerror();
    }
};

struct SymbolNotFound : public std::runtime_error
{
    using runtime_error::runtime_error;
};

template <typename RetType, typename... Args>
RetType CallFunction(std::string lib_name,
                     std::string function_name,
                     Args... args)
{

    /* Try to open the provided library */
    void * lib_handle = dlopen(lib_name.c_str(), RTLD_NOW);

    /* Not able to open so, report error */
    if(!lib_handle)
    {
        throw LibNotFound();
    }
    else
    {

        /* Typedef for the factory type */
        typedef RetType (*FactoryType)(Args... args);

        /* Try to obtain the address of the factory */
        FactoryType function = reinterpret_cast<FactoryType>(dlsym(lib_handle,
                                                                  function_name.c_str())
                                                            );

        const char * error = dlerror();
        if(error != nullptr)
        {
            throw SymbolNotFound(error);
        }

        return function(args...);

    }

}

#endif // LOAD_OBJECT_H


#endif // DYNAMICLOADING_H
