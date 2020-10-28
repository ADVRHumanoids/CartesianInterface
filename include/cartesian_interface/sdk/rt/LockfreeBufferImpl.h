#include <cartesian_interface/CartesianInterfaceImpl.h>
#include "utils/spsc_queue_ci.hpp"
#include "TaskRt.h"

namespace XBot { namespace Cartesian {
  
    class LockfreeBufferImpl : public CartesianInterfaceImpl
    {
        
        typedef std::function<void(CartesianInterface*)> CallbackType;
      
    public:
        
        typedef std::shared_ptr<LockfreeBufferImpl> Ptr;
        
        LockfreeBufferImpl(CartesianInterfaceImpl * ci, Context::Ptr context);
        
        void callAvailable(CartesianInterface * ci);
        
        void pushState(CartesianInterface const * ci, ModelInterface const * model);

        void updateState();
        
        virtual bool reset(double time);
        
        virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);
        
    private:
        
        static const int CALL_QUEUE_SIZE = 128;
        static const int DEFAULT_QUEUE_SIZE = 10;
        
        template <typename T, int N = DEFAULT_QUEUE_SIZE>
        using LockFreeQueue = boost::lockfree::spsc_queue<T, boost::lockfree::capacity<N>>;
        
        LockFreeQueue<CallbackType, CALL_QUEUE_SIZE> _cb_queue;

        Eigen::VectorXd _q_tmp;
        LockFreeQueue<Eigen::VectorXd> _model_state_queue;
        
        Eigen::VectorXd _q_tmp_read;
        ModelInterface::Ptr _model;
        
        std::vector<TaskRt::Ptr> _tasks;
        
        
    };
    

    
} }
