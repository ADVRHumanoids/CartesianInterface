#ifndef ROSUTILS_H
#define ROSUTILS_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <std_msgs/msg/float64_multi_array.hpp>

template <typename SrvType, typename Obj>
auto create_service(rclcpp::Node::SharedPtr node,
                    std::string name,
                    bool (Obj::* mem_fn)(typename SrvType::Request::ConstSharedPtr, typename SrvType::Response::SharedPtr),
                    Obj * obj)
{
    auto cb = [obj, mem_fn](typename SrvType::Request::ConstSharedPtr req,
                            typename SrvType::Response::SharedPtr res)
    {
        return (obj->*mem_fn)(req, res);
    };

    return node->create_service<SrvType>(name, cb);
}

template <typename MsgType, typename Obj>
auto create_subscription(rclcpp::Node::SharedPtr node,
                    std::string name,
                    void (Obj::* mem_fn)(typename MsgType::ConstSharedPtr),
                         Obj * obj, rclcpp::QoS qos = rclcpp::QoS(1))
{
    auto cb = [obj, mem_fn](typename MsgType::ConstSharedPtr msg)
    {
        return (obj->*mem_fn)(msg);
    };

    return node->create_subscription<MsgType>(name, qos, cb);
}

template <typename Srv>
typename Srv::Response call_service(rclcpp::Node::SharedPtr node,
                                    typename rclcpp::Client<Srv>::SharedPtr srv,
                                    std::shared_ptr<typename Srv::Request> req,
                                    auto timeout)
{

    auto fut = srv->async_send_request(req);


}

template<class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::msg::Float64MultiArray &m)
{
    if (m.layout.dim.size() != 2)
        m.layout.dim.resize(2);
    m.layout.dim[0].stride = e.rows() * e.cols();
    m.layout.dim[0].size = e.rows();
    m.layout.dim[1].stride = e.cols();
    m.layout.dim[1].size = e.cols();
    if ((int) m.data.size() != e.size())
        m.data.resize(e.size());
    int ii = 0;
    for (int i = 0; i < e.rows(); ++i)
        for (int j = 0; j < e.cols(); ++j)
            m.data[ii++] = e.coeff(i, j);
}


#endif // ROSUTILS_H
