#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "world_connecter");
    ros::NodeHandle nh, nh_priv("~");
    
    if(argc != 5)
    {
        std::cout << "Usage: " << argv[0] << " world_1 world_2 root_1 root_2" << std::endl;
        exit(1);
    }
    
    std::string world_1(argv[1]), world_2(argv[2]), root_1(argv[3]), root_2(argv[4]);
    
    tf::TransformListener listener;
    listener.waitForTransform(world_1, root_1, ros::Time(0), ros::Duration(10.0) );
    listener.waitForTransform(world_2, root_2, ros::Time(0), ros::Duration(10.0) );
    
    tf::TransformBroadcaster br;

    double rate_value = nh_priv.param("rate", 25);
    ros::Rate rate(rate_value);
    
    while(ros::ok())
    {
        
        tf::StampedTransform w_T_r1, w_T_r2;
        
        try{
            listener.lookupTransform(root_1, world_1,  
                                     ros::Time(0), w_T_r1);
            
            listener.lookupTransform(root_2, world_2,  
                                     ros::Time(0), w_T_r2);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
        
        auto r2_T_r1 = w_T_r2.inverseTimes(w_T_r1);
        
        br.sendTransform(tf::StampedTransform(r2_T_r1, ros::Time::now(), root_1, root_2));
        
        rate.sleep();
    }
    
    return EXIT_SUCCESS;
    
}