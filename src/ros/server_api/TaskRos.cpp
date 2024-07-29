#include "ros/server_api/TaskRos.h"
#include "utils/DynamicLoading.h"
#include "fmt/format.h"

#include "../utils/RosUtils.h"

#include "ros/server_api/CartesianRos.h"
#include "ros/server_api/PosturalRos.h"
#include "ros/server_api/InteractionRos.h"


using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;


TaskRos::TaskRos(TaskDescription::Ptr task,
                 RosContext::Ptr context):
    _task(task),
    task_name(task->getName()),
    _model(context->ci_context()->model()),
    _ctx(context)
{
    auto n = _ctx->node();

    rclcpp::ServiceBase::SharedPtr srv;

    srv = ::create_service<GetTaskInfo>(n,
                task_name + "/get_task_properties",
                &TaskRos::get_task_info_cb, this);

    _srv.push_back(srv);

    srv = ::create_service<SetLambda>(n,
               task_name + "/set_lambda",
               &TaskRos::set_lambda_cb, this);

    _srv.push_back(srv);

    srv = ::create_service<SetLambda2>(n,
               task_name + "/set_lambda2",
               &TaskRos::set_lambda2_cb, this);

    _srv.push_back(srv);

    srv = ::create_service<SetWeight>(n,
               task_name + "/set_weight",
               &TaskRos::set_weight_cb, this);

    _srv.push_back(srv);

    srv = ::create_service<SetTaskActive>(n,
               task_name + "/set_active",
               &TaskRos::set_active_cb, this);

    _srv.push_back(srv);

    _task_changed_pub = n->create_publisher<std_msgs::msg::String>(
               task_name + "/task_changed_event", 10);

    _task_err_pub = n->create_publisher<std_msgs::msg::Float64MultiArray>(
               task_name + "/task_error", 10);

    _task_info_pub = n->create_publisher<TaskInfo>(
               task_name + "/task_properties", 10);

    _type_hierarchy.push_back("Task");
}

void TaskRos::run(rclcpp::Time time)
{
   /* Publish task error */
   if(_task->getTaskError(_err))
   {
       std_msgs::msg::Float64MultiArray msg;

       matrixEigenToMsg(_err, msg);
       _task_err_pub->publish(msg);
   }

   /* Publish task info */
   auto info_req = std::make_shared<GetTaskInfo::Request>();
   auto info_res = std::make_shared<GetTaskInfo::Response>();
   get_task_info_cb(info_req, info_res);

   TaskInfo info_msg;
   info_msg.activation_state = info_res->activation_state;
   info_msg.disabled_joints = info_res->disabled_joints;
   info_msg.indices = info_res->indices;
   info_msg.lambda1 = info_res->lambda1;
   info_msg.lambda2 = info_res->lambda2;
   info_msg.lib_name = info_res->lib_name;
   info_msg.name = info_res->name;
   info_msg.size = info_res->size;
   info_msg.type = info_res->type;
   info_msg.weight = info_res->weight;

   _task_info_pub->publish(info_msg);

}

bool TaskRos::onActivationStateChanged()
{
   notifyTaskChanged("ActivationState");

   return true;
}


bool TaskRos::get_task_info_cb(GetTaskInfo::Request::ConstSharedPtr req,
                               GetTaskInfo::Response::SharedPtr res)
{
   res->name = _task->getName();

   res->type.assign(_type_hierarchy.begin(), _type_hierarchy.end());

   res->size = _task->getSize();

   res->lambda1 = _task->getLambda();

   res->lambda2 = _task->getLambda2();

   res->indices.assign(_task->getIndices().begin(),
                      _task->getIndices().end());

   res->disabled_joints = _task->getDisabledJoints();

   res->activation_state = EnumToString(_task->getActivationState());

   res->weight.resize(res->size*res->size);

   Eigen::MatrixXf::Map(res->weight.data(),
                        res->size,
                        res->size) = _task->getWeight().cast<float>();

   res->lib_name = _task->getLibName();

   return true;
}

bool TaskRos::set_lambda_cb(SetLambda::Request::ConstSharedPtr req,
                            SetLambda::Response::SharedPtr res)
{
   if(req->lambda1 >= 0.0 && req->lambda1 <= 1.0)
   {
       _task->setLambda(req->lambda1);
       res->message = fmt::format("Successfully set lambda = {} to task '{}'",
                                 req->lambda1, _task->getName());
       res->success = true;

       return true;
   }

   res->message = fmt::format("Unable to set lambda = {} to task '{}': value not in range [0, 1]",
                             req->lambda1, _task->getName());
   res->success = false;

   return true;
}

bool TaskRos::set_lambda2_cb(SetLambda2::Request::ConstSharedPtr req,
                             SetLambda2::Response::SharedPtr res)
{
   if(req->auto_lambda2)
   {
       _task->setLambda2(-1);
       res->message = fmt::format("Successfully set lambda2 = {} to task '{}'",
                                 -1, _task->getName());
       res->success = true;

       return true;
   }

   if(req->lambda2 >= 0.0 && req->lambda2 <= 1.0)
   {
       _task->setLambda2(req->lambda2);
       res->message = fmt::format("Successfully set lambda2 = {} to task '{}'",
                                 req->lambda2, _task->getName());
       res->success = true;

       return true;
   }


   res->message = fmt::format("Unable to set lambda2 = {} to task '{}': value not in range [0, 1]",
                             req->lambda2, _task->getName());
   res->success = false;

   return true;
}

bool TaskRos::set_weight_cb(SetWeight::Request::ConstSharedPtr req,
                            SetWeight::Response::SharedPtr res)
{
   Eigen::MatrixXd w;
   w.setIdentity(_task->getSize(), _task->getSize());

   if(req->weight.size() == 1)
   {
       w *= req->weight[0];
   }

   else if(req->weight.size() == _task->getSize())
   {
       w = Eigen::VectorXf::Map(req->weight.data(), req->weight.size()).cast<double>().asDiagonal();
   }

   else if(req->weight.size() == _task->getSize()*_task->getSize())
   {
       w = Eigen::MatrixXf::Map(req->weight.data(),
                                _task->getSize(),
                                _task->getSize()).cast<double>();
   }
   else
   {
       res->success = false;
       res->message = fmt::format("Unable to set weight to task '{}': size should be either 1, n, or n^2",
                                 _task->getName());
       return true;
   }

   _task->setWeight(w);

   res->success = true;
   res->message = fmt::format("Successfully set weight to task '{}'",
                             _task->getName());

   return true;

}

bool TaskRos::set_active_cb(SetTaskActive::Request::ConstSharedPtr req,
                            SetTaskActive::Response::SharedPtr res)
{
   ActivationState req_state;

   if(req->activation_state)
   {
       req_state = ActivationState::Enabled;
   }
   else
   {
       req_state = ActivationState::Disabled;
   }

   res->success = _task->setActivationState(req_state);

   if(res->success)
   {
       res->message = fmt::format("Succesfully set activation state to '{}' for task '{}'",
                                 EnumToString(req_state), _task->getName());
   }
   else
   {
       res->message = fmt::format("Unable to set activation state to '{}' for task '{}'",
                                 EnumToString(req_state), _task->getName());
   }

   return true;
}

RosApiNotFound::RosApiNotFound(std::string message): _msg(message)
{}

const char * RosApiNotFound::what() const noexcept
{
   return _msg.c_str();
}


TaskRos::Ptr TaskRos::MakeInstance(TaskDescription::Ptr task,
                                  RosContext::Ptr context)
{
    Ptr rosapi_shared_ptr;

   /* Try all supported dynamic casts, from the most derived to the least derived class */
   if(auto inter = std::dynamic_pointer_cast<InteractionTask>(task))
   {
       // ros_adapter = new InteractionRos(inter, context);
   }
   else if(auto cart = std::dynamic_pointer_cast<CartesianTask>(task))
   {
       rosapi_shared_ptr = std::make_shared<CartesianRos>(cart, context);
   }
   else if(auto post = std::dynamic_pointer_cast<PosturalTask>(task))
   {
       // ros_adapter = new PosturalRos(post, context);
   }
   else if(!task->getLibName().empty()) /* Otherwise, construct plugin, or fallback to generic Task interface */
   {
       try
       {
           auto ros_adapter = CallFunction<TaskRos*>(task->getLibName(),
                                                "create_cartesio_" + task->getType() + "_ros_api",
                                                task,
                                                context,
                                                detail::Version CARTESIO_ABI_VERSION);

           rosapi_shared_ptr.reset(ros_adapter);
       }
       catch(LibNotFound&)
       {
           fmt::print("Unable to construct TaskRos instance for task '{}': "
                      "lib '{}' not found for task type '{}' \n",
                      task->getName(), task->getLibName(), task->getType());

       }
       catch(SymbolNotFound&)
       {
           fmt::print("Unable to construct TaskRos instance for task '{}': "
                      "factory not found for task type '{}' \n",
                      task->getName(), task->getType());

       }
   }

   if(!rosapi_shared_ptr)
   {
       rosapi_shared_ptr = std::make_shared<TaskRos>(task, context);
   }

   if(!rosapi_shared_ptr->initialize())
   {
       throw std::runtime_error(fmt::format("Unable to initialize TaskRos instance for task '{}'",
                                            task->getName()));
   }

   return rosapi_shared_ptr;
}

bool TaskRos::initialize()
{
   _task->registerObserver(shared_from_this());

   return true;
}

void TaskRos::notifyTaskChanged(const std::string & message)
{
    std_msgs::msg::String msg;
    msg.data = message;
    _task_changed_pub->publish(msg);
}

void TaskRos::registerType(const std::string & type)
{
   _type_hierarchy.push_front(type);
}

std::string TaskRos::RosApiPluginName(TaskDescription::Ptr task)
{
   return "libcartesio_ros_api_" + task->getType() + ".so";
}
