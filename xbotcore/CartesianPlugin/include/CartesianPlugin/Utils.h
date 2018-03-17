#ifndef CartesianPlugin_UTILS_H_
#define CartesianPlugin_UTILS_H_

#include <XCM/XBotControlPlugin.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

namespace XBot { namespace Cartesian { namespace Utils {
 
    class SyncFromIO
    {
        
    public:
        
        typedef std::shared_ptr<SyncFromIO> Ptr;
        
        SyncFromIO(std::string shared_object_name, SharedMemory::Ptr shared_memory);
        
        bool try_sync(double time, CartesianInterfaceImpl::Ptr ci, ModelInterface::ConstPtr model);
        
        bool try_reset(ModelInterface::ConstPtr model);
        
        
        
    private:
        
        SharedObject<CartesianInterfaceImpl::Ptr> _ci_shobj;
        CartesianInterfaceImpl::Ptr _ci_nrt;
    };
    
} } }



inline XBot::Cartesian::Utils::SyncFromIO::SyncFromIO(std::string shared_object_name, SharedMemory::Ptr shared_memory):
    _ci_shobj( shared_memory->getSharedObject<CartesianInterfaceImpl::Ptr>(shared_object_name) )
{

}

inline bool XBot::Cartesian::Utils::SyncFromIO::try_reset(XBot::ModelInterface::ConstPtr model)
{
    if(_ci_nrt || (_ci_shobj.try_get(_ci_nrt) && _ci_nrt))
    {
        if(_ci_shobj.get_mutex()->try_lock())
        {
                _ci_nrt->getModel()->syncFrom(*model);
                _ci_nrt->reset();
                _ci_shobj.get_mutex()->unlock();
                return true;
        }
    }
    
    return false;
}

inline bool XBot::Cartesian::Utils::SyncFromIO::try_sync(double time, 
                                                  XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                                                  XBot::ModelInterface::ConstPtr model)
{
    if(_ci_nrt || (_ci_shobj.try_get(_ci_nrt) && _ci_nrt))
    {
        if(_ci_shobj.get_mutex()->try_lock())
        {
            _ci_nrt->getModel()->syncFrom(*model);
            _ci_nrt->update(time, 0.0);
            
             ci->syncFrom(_ci_nrt);
            _ci_shobj.get_mutex()->unlock();
            
            return true;
        }
    }
    
    return false;
}


#endif