/*
 * Copyright (C) 2018 IIT-ADVR
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __XBOT_CARTESIAN_PROBLEM_DESC_H__
#define __XBOT_CARTESIAN_PROBLEM_DESC_H__

#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Supported task types enum
     */
    enum class TaskType { Cartesian, Postural, Com, Gaze };
    
    
    /**
     * @brief Base class for task descriptions, which contains
     * properties that are shared among all tasks.
     */
    struct TaskDescription {
        
        /**
         * @brief Task type
         */
        TaskType type;
        
        /**
         * @brief Task literal ID
         */
        std::string name;
        
        /**
         * @brief Task weight. Inside an aggregated task, each component
         * is weighted according to this variable, that is therefore useful
         * to model soft priorities. It MUST be a positive-definite symmetric
         * matrix of size equal to the task size.
         */
        Eigen::MatrixXd weight;
        
        /**
         * @brief Vector of indices representing a subtask of the original task.
         * The resulting task size is equal to indices.size(). When modifying
         * this variable manually, the weight matrix must be changed as well to 
         * reflect the size change. Otherwise, use operator%.
         */
        std::vector<int> indices;
        
        /**
         * @brief Feedback gain on the task error. Lower values
         * make the cartesian controller less reactive.
         */
        double lambda;
        
        
        typedef std::shared_ptr<TaskDescription> Ptr;
        typedef std::shared_ptr<const TaskDescription> ConstPtr;
        
        TaskDescription() = default;
        TaskDescription(TaskType type, std::string name, int size);
        
        virtual ~TaskDescription(){}
        
    private:
        
        
        
    };
    
    /**
     * @brief Typedef for a vector of tasks representing an aggregated task.
     */
    typedef std::vector<TaskDescription::Ptr> AggregatedTask;
    
    /**
     * @brief Typedef for a vector of aggregated tasks, representing
     * a hierachical optimization problem.
     */
    typedef std::vector<AggregatedTask> Stack;
    
    /**
     * @brief Description of a Gaze task
     */
    struct GazeTask : TaskDescription {

        std::string base_link;

        typedef std::shared_ptr<GazeTask> Ptr;
        typedef std::shared_ptr<const GazeTask> ConstPtr;

        GazeTask() = default;
        GazeTask(std::string base_link = "world");
    };

    /**
     * @brief Make a Gaze task and return a shared pointer
     */
    GazeTask::Ptr MakeGaze(std::string base_link = "world");

    /**
     * @brief Dynamic cast a generic task to a GazeTask
     *
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a GazeTask)
     */
    GazeTask::Ptr GetAsGaze(TaskDescription::Ptr task);
    
    /**
     * @brief Description of a cartesian task
     */
    struct CartesianTask : TaskDescription {
        
        /**
         * @brief The task controls the relative motion of distal_link w.r.t base_link
         */
        std::string base_link, distal_link;

        /**
         * @brief Parameter that weights orientation errors w.r.t. position errors.
         * For example, setting it to orientation_gain = 2.0 means that an error of
         * 2 rad is recovered in the same time as an error of 1 meter.
         */
        double orientation_gain;
        
        typedef std::shared_ptr<CartesianTask> Ptr;
        typedef std::shared_ptr<const CartesianTask> ConstPtr;
        
        CartesianTask() = default;
        CartesianTask(std::string distal_link, std::string base_link = "world");
        
        
    };
    
    /**
     * @brief Make a cartesian task and return a shared pointer
     */
    CartesianTask::Ptr MakeCartesian(std::string distal_link, std::string base_link = "world");

    /**
     * @brief Dynamic cast a generic task to a CartesianTask
     * 
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a CartesianTask)
     */
    CartesianTask::Ptr GetAsCartesian(TaskDescription::Ptr task);
    
    
    /**
     * @brief Description of a center of mass task
     */
    struct ComTask : TaskDescription {
        
        typedef std::shared_ptr<ComTask> Ptr;
        typedef std::shared_ptr<const ComTask> ConstPtr;
        
        ComTask();
        
    };
    
    /**
     * @brief Make a CoM task and return a shared pointer
     */
    ComTask::Ptr MakeCom();
    
    /**
     * @brief Dynamic cast a generic task to a ComTask
     * 
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a ComTask)
     */
    ComTask::Ptr GetAsCom(TaskDescription::Ptr task);
    
    
    /**
     * @brief Description of a postural (i.e. joint space) task
     * 
     */
    struct PosturalTask : TaskDescription {
        
        typedef std::shared_ptr<PosturalTask> Ptr;
        typedef std::shared_ptr<const PosturalTask> ConstPtr;
        
        /**
         * @brief Construct a postural task from the number of robot dofs (including 6 virtual joints)
         * 
         * @param ndof The number of robot dofs (including 6 virtual joints)
         */
        PosturalTask(int ndof);
        
    };
    
    /**
    * @brief Construct a postural task and return a shared pointer
    * 
    * @param ndof The number of robot dofs (including 6 virtual joints)
    */
    PosturalTask::Ptr MakePostural(int ndof);
    
    /**
     * @brief Dynamic cast a generic task to a PosturalTask
     * 
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a PosturalTask)
     */
    PosturalTask::Ptr GetAsPostural(TaskDescription::Ptr task);
    
    /**
     * @brief Supported constraint types.
     */
    enum class ConstraintType { JointLimits, VelocityLimits };
    
    /**
     * @brief Base class for the description of a constraint.
     */
    struct ConstraintDescription {
        
        ConstraintType type;
        
        typedef std::shared_ptr<ConstraintDescription> Ptr;
        typedef std::shared_ptr<const ConstraintDescription> ConstPtr;
        
        ConstraintDescription() = default;
        ConstraintDescription(ConstraintType type);
        
        virtual ~ConstraintDescription(){}
        
    };
    
    /**
    * @brief Construct a joint limits constraint and return a shared pointer
    */
    ConstraintDescription::Ptr MakeJointLimits();
    
    /**
    * @brief Construct a joint velocity limits constraint and return a shared pointer
    */
    ConstraintDescription::Ptr MakeVelocityLimits();
    

    /**
     * @brief Description of a hierarchical optimization problem in terms of
     *  a vector of aggregated tasks (one for each priority level) and a vector
     * of global constraints.
     * 
     * The main purpose of this class is to provide a way to specify these two
     * quantities from a YAML node.
     */
    class ProblemDescription {
        
    public:
        
        /**
         * @brief Construct from a single task (i.e. single priority level, single task)
         */
        ProblemDescription(TaskDescription::Ptr task);
        
        /**
         * @brief Construct from a single aggregated task (i.e. single priority level, multiple tasks)
         */
        ProblemDescription(AggregatedTask task);
        
        /**
         * @brief Construct from a stack (i.e. multiple priority level, multiple tasks)
         */
        ProblemDescription(Stack stack);
        
        /**
         * @brief Construct from a YAML description of the IK problem. The required YAML
         * formatting is described at 
         * 
         * https://github.com/ADVRHumanoids/CartesianInterface/wiki/Get-started%21#writing-an-ik-problem-for-your-robot
         * 
         * @param yaml_node YAML node containing a problem_description node
         * @param model Model of the robot to be controlled (used to retrieve information like the dof number)
         */
        ProblemDescription(YAML::Node yaml_node, ModelInterface::ConstPtr model);
        
        /**
         * @brief Adds one constraint to the IK problem
         */
        ProblemDescription& operator<<(ConstraintDescription::Ptr constraint);
        
        /**
         * @brief Number of hierachy levels
         */
        int getNumTasks() const;
        
        /**
         * @brief Gets the id-th aggregated task representing the id-th 
         * hierachy level
         */
        AggregatedTask getTask(int id) const;
        
        /**
         * @brief Reference to the vector of global constraints
         */
        const std::vector< ConstraintDescription::Ptr >& getBounds() const;
        
        /**
         * @brief Getter to the "solver_options" node inside the YAML file.
         * If such field did not exist, a null node is returned
         * 
         * @return const YAML::Node&
         */
        const YAML::Node& getSolverOptions() const;
        
    private:
        
        std::vector< std::vector<TaskDescription::Ptr> > _stack;
        std::vector< ConstraintDescription::Ptr > _bounds;
        YAML::Node _solver_options_node;
        
    };
    
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             TaskDescription::Ptr task_2);
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(AggregatedTask task_1, 
                             TaskDescription::Ptr task_2);
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             AggregatedTask task_2);
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(AggregatedTask task_1, 
                             AggregatedTask task_2);
    
    /**
     * @brief Apply a weight matrix to a task
     * 
     * @param weight Symmetric positive definite matrix that is applied on the left to 
     * the task weight
     * @param task Task that we want to modify the weight of
     * @return The same pointer that was provided as input
     */
    TaskDescription::Ptr operator*(Eigen::Ref<const Eigen::MatrixXd> weight, TaskDescription::Ptr task);
    
    /**
     * @brief Apply a subtask selection to a task. The function also 
     * adjusts the task weight matrix.
     */
    TaskDescription::Ptr operator%(std::vector<int> indices, TaskDescription::Ptr task);
    
    // TBD refactor to enable more than two levels
    Stack operator/(TaskDescription::Ptr task_1, 
                    TaskDescription::Ptr task_2);
    
    Stack operator/(TaskDescription::Ptr task_1, 
                    AggregatedTask task_2);
    
    Stack operator/(AggregatedTask task_1, 
                    TaskDescription::Ptr task_2);
    
    Stack operator/(AggregatedTask task_1, 
                    AggregatedTask task_2);
    
} }


#endif







