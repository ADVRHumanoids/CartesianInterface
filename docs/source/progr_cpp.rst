Programmatic API (C++)
======================
From C++ code, it is possible to

 - create an instance of the ``RosClient`` class in order to interact with a remotely running
   instance of CartesIO
 - create an instance of the ``CartesianInterfaceImpl`` class in order to actually set up and
   solve an IK problem, or..

Once one of these objects is created, their API is essentially shared.
You can check a reference (doxygen generated) of the C++ API by following
:ref:`this link<cppref>`.


A minimal example (ROS client)
------------------------------

.. code-block:: c++

        #include <cartesian_interface/ros/RosClient.h>

        using namespace XBot::Cartesian;

        int main(int argc, char ** argv)
        {
            RosClient cli;
            auto task = cli.getTask("arm1_8");

            // ..use task
            task->setLambda(0.5);

            // ..convert to Cartesian task
            auto task_cartesian = std::dynamic_pointer_cast<CartesianTask>(task);

            // if conversion was successful...
            if(task_cartesian)
            {
                Eigen::Affine3d Ttgt;
                double time = 5.0;

                // fill Ttgt...
                task_cartesian->getPoseReference(Ttgt);
                Ttgt.translation().x() += 0.2;

                // command reaching motion
                task_cartesian->setPoseTarget(Ttgt, time);

                // sleep time
                const double sleep_dt = 0.1;

                // wait until motion started
                while(task_cartesian->getTaskState() == State::Online)
                {
                    cli.update(0, 0);
                    ros::Duration(sleep_dt).sleep();
                }

                std::cout << "Motion started" << std::endl;

                // wait until motion completed
                while(task_cartesian->getTaskState() == State::Reaching)
                {
                    cli.update(0, 0);
                    ros::Duration(sleep_dt).sleep();
                }

                std::cout << "Motion completed" << std::endl;
            }
        }

A minimal `CMakeLists.txt` to compile the snippet above is the following


.. code-block:: cmake

        cmake_minimum_required(VERSION 3.5)

        project(cartesio_example_test)
        find_package(catkin REQUIRED COMPONENTS cartesian_interface)

        add_compile_options(-std=c++11)

        include_directories(${catkin_INCLUDE_DIRS})

        add_executable(exampleros exampleros.cpp)
        target_link_libraries(exampleros ${catkin_LIBRARIES})



A minimal example (Solver)
------------------------------
An example of embedding the actual solver is way more complex, due mainly to the
setup code needed.

Part I: setup
^^^^^^^^^^^^^
We here retrieve the minimal configuration file set, which is composed of

 - a URDF file (for this example, we assume it to be available at `URDF_PATH`)
 - an SRDF file (`SRDF_PATH`)
 - a YAML file with the ik problem description (`IK_PB_PATH`)

Based on this, we create all required classes, which include:

 - an ``XBot::ModelInterface`` object to hold the kinematic/dynamic model of the system
 - the ``XBot::Cartesian::ProblemDescription`` object which describes the ik problem
 - the ``XBot::Cartesian::CartesianInterfaceImpl`` object which is the actual solver

.. code-block:: c++

    #include <cartesian_interface/CartesianInterfaceImpl.h>
    #include <thread>

    using namespace XBot::Cartesian;

    int main(int argc, char **argv)
    {
        // an option structure which is needed to make a model
        XBot::ConfigOptions xbot_cfg;

        // set the urdf and srdf path..
        xbot_cfg.set_urdf_path(URDF_PATH);
        xbot_cfg.set_srdf_path(SRDF_PATH);

        // the following call is needed to generate some default joint IDs
        xbot_cfg.generate_jidmap();

        // some additional parameters..
        xbot_cfg.set_parameter("is_model_floating_base", true);
        xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

        // and we can make the model class
        auto model = XBot::ModelInterface::getModel(xbot_cfg);

        // initialize to a homing configuration
        Eigen::VectorXd qhome;
        model->getRobotState("home", qhome);
        model->setJointPosition(qhome);
        model->update();

        // before constructing the problem description, let us build a
        // context object which stores some information, such as
        // the control period
        const double dt = 0.01;
        auto ctx = std::make_shared<Context>(
                    std::make_shared<Parameters>(dt),
                    model
                );

        // load the ik problem given a yaml file
        auto ik_pb_yaml = YAML::LoadFile(IK_PB_PATH);
        ProblemDescription ik_pb(ik_pb_yaml, ctx);

        // we are finally ready to make the CartesIO solver "OpenSot"
        auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                           ik_pb, ctx
                                                           );

Part II: control loop
^^^^^^^^^^^^^^^^^^^^^
We here implement a simplistic finite state machine to command the ``left_hand``
Cartesian task to a target pose, and then exit. At all iterations, it is necessary to:

 - call ``CartesianInterface::update()`` to compute the ik, which will set the optimized
   motion (in terms of joint velocity or acceleration) to the model
 - integrate such motion to update the joint state

Upon succesfull exit, some `.mat` files will be available for inspection, as they are
auto-generated by CartesIO.

.. code-block:: c++

     int current_state = 0; // hand-crafted finite state machine!
     double time = 0;
     Eigen::VectorXd q, qdot, qddot;
     while(true)
     {
         if(current_state == 0) // here we command a reaching motion
         {
             std::cout << "Commanding left hand forward 0.3m in 3.0 secs" << std::endl;

             larm_cartesian->getPoseReference(Tref);
             Tref.translation()[0] += 0.3;
             double target_time = 3.0;
             larm_cartesian->setPoseTarget(Tref, target_time);

             current_state++;
         }

         if(current_state == 1) // here we check that the reaching started
         {
             if(larm_cartesian->getTaskState() == State::Reaching)
             {
                 std::cout << "Motion started!" << std::endl;
                 current_state++;
             }
         }

         if(current_state == 2) // here we wait for it to be completed
         {
             if(larm_cartesian->getTaskState() == State::Online)
             {
                 Eigen::Affine3d T;
                 larm_cartesian->getCurrentPose(T);

                 std::cout << "Motion completed, final error is " <<
                               (T.inverse()*Tref).translation().norm() << std::endl;

                 current_state++;
             }
         }

         if(current_state == 3) // here we wait the robot to come to a stop
         {
             std::cout << "qdot norm is " << qdot.norm() << std::endl;
             if(qdot.norm() < 1e-3)
             {
                 std::cout << "Robot came to a stop, press ENTER to exit.. \n";
                 std::cin.ignore();
                 current_state++;
             }

         }

         if(current_state == 4) break;

         // update and integrate model state
         solver->update(time, dt);

         model->getJointPosition(q);
         model->getJointVelocity(qdot);
         model->getJointAcceleration(qddot);

         q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
         qdot += dt * qddot;

         model->setJointPosition(q);
         model->setJointVelocity(qdot);
         model->update();


         // roughly loop at 100 Hz
         std::this_thread::sleep_for(std::chrono::duration<double>(dt));
         time += dt;
     }


How to compile
--------------

.. code-block:: cmake

        find_package(cartesian_interface REQUIRED)

        include_directories(${cartesian_interface_INCLUDE_DIRS})

        add_executable(cartesio_solver cartesio_solver.cpp)

        target_link_libraries(cartesio_solver ${cartesian_interface_LIBRARIES})
