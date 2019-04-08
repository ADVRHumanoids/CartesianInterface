#ifndef CartesianPlugin_UTILS_H_
#define CartesianPlugin_UTILS_H_

#include <XCM/XBotControlPlugin.h>
#include <cartesian_interface/utils/LockfreeBufferImpl.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>

namespace XBot { namespace Cartesian { namespace Utils {
 
    class SyncFromIO
    {
        
    public:
        
        typedef std::shared_ptr<SyncFromIO> Ptr;
        
        SyncFromIO(XBot::Handle::Ptr handle, 
                XBot::Cartesian::CartesianInterface::ConstPtr ci, 
                XBot::ModelInterface::ConstPtr model,
                std::string tf_prefix = "ci",
                std::string ros_namespace = "cartesian"
                );
        
        void set_solver_active(bool is_active);
        
        void send(CartesianInterfaceImpl::Ptr ci, ModelInterface::ConstPtr model);
        
        void receive(CartesianInterfaceImpl::Ptr ci);
        
        
        
    private:
        
        LockfreeBufferImpl::Ptr _ci_buf;
        XBot::SharedObject<LockfreeBufferImpl::Ptr> _ci_buf_shobj;
        XBot::SharedObject<std::atomic<bool>> _ci_running_shobj;
        XBot::SharedObject<RosServerClass::Ptr> _ci_ros_shobj;
    };
    
} } }



inline XBot::Cartesian::Utils::SyncFromIO::SyncFromIO(XBot::Handle::Ptr handle, 
                                                      XBot::Cartesian::CartesianInterface::ConstPtr ci, 
                                                      XBot::ModelInterface::ConstPtr model,
                                                      std::string tf_prefix,
                                                      std::string ros_namespace
                                                     )
{
    /* Model to be used on the nrt side */
    auto ros_model = ModelInterface::getModel(handle->getPathToConfigFile());
    
    /* Lockfree buffer for wait-free communication between RT <-> NRT */
    _ci_buf = std::make_shared<LockfreeBufferImpl>(ci.get(), ros_model);
    _ci_buf->pushState(ci.get(), model.get());
    _ci_buf->updateState();
    
    /* Ros API server */
    RosServerClass::Options opt;
    opt.tf_prefix = tf_prefix;
    opt.ros_namespace = ros_namespace;
    auto ci_ros = std::make_shared<RosServerClass>(_ci_buf, ros_model, opt);
    
    /* Initialize required variables in shared memory */
    auto shmem = handle->getSharedMemory();
    
    _ci_buf_shobj = shmem->getSharedObject<LockfreeBufferImpl::Ptr>("/xbotcore/ci_buffer");
    _ci_buf_shobj.set(_ci_buf);
    
    _ci_running_shobj = shmem->getSharedObject<std::atomic<bool>>("/xbotcore/ci_running");
    _ci_running_shobj.get_object_ptr()->store(false);
    
    _ci_ros_shobj = shmem->getSharedObject<RosServerClass::Ptr>("/xbotcore/ci_ros_server_class");
    _ci_ros_shobj.set(ci_ros);
}

inline void XBot::Cartesian::Utils::SyncFromIO::receive(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci)
{
    _ci_buf->callAvailable(ci.get());
}

inline void XBot::Cartesian::Utils::SyncFromIO::send(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci, XBot::ModelInterface::ConstPtr model)
{
    _ci_buf->pushState(ci.get(), model.get());
}

inline void XBot::Cartesian::Utils::SyncFromIO::set_solver_active(bool is_active)
{
    _ci_running_shobj.get_object_ptr()->store(is_active);
}


#endif
