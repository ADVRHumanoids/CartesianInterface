#ifndef __XBOT_CARTESIAN_INTERFACE_UTILS_LOAD_OBJ_HPP__
#define __XBOT_CARTESIAN_INTERFACE_UTILS_LOAD_OBJ_HPP__

#include <cstdio>
#include <dlfcn.h>

#include <string>
#include <memory>

namespace XBot { namespace Cartesian { namespace Utils {

template <typename DescType, typename... Args>
std::unique_ptr<DescType> LoadObject(std::string lib_name, 
                                     std::string factory_name, 
                                     Args... args)
{
    
    /* Try to open the provided library */
    void * lib_handle = dlopen(lib_name.c_str(), RTLD_NOW);
    
    /* Not able to open so, report error */
    if(!lib_handle)
    {
        fprintf(stderr, "%s\n", dlerror());
        return nullptr;
    } 
    else 
    {

        /* Typedef for the factory type */
        typedef DescType * (*FactoryType)(Args... args);
        
        /* Try to obtain the address of the factory */
        FactoryType factory = reinterpret_cast<FactoryType>(dlsym(lib_handle, 
                                                                  factory_name.c_str())
                                                           );
        
        const char * error = dlerror();
        if(error != nullptr)
        {
            fprintf(stderr, 
                    "Unable to load factory '%s', error '%s'\n", 
                    factory_name.c_str(),
                    error);
            
            return nullptr;
        }

        DescType * instance = factory(args...);
        
        if(instance != nullptr) 
        {
            return std::unique_ptr<DescType>(instance);
        }
        
        fprintf(stderr, 
                "Error in loading library '%s': obtained pointer is null\n", 
                lib_name.c_str());

    }

    return nullptr;
    
}

} } }

#endif
