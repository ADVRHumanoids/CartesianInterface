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

#include <vector>
#include <memory>
#include <list>
#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace XBot { namespace Cartesian {
    
    enum class TaskType { Cartesian, Postural, Com };
    
    struct TaskDescription {
        
        TaskType type;
        Eigen::MatrixXd weight;
        std::vector<int> indices;
        double lambda;
        
        typedef std::shared_ptr<TaskDescription> Ptr;
        typedef std::shared_ptr<const TaskDescription> ConstPtr;
        
        TaskDescription() = default;
        TaskDescription(TaskType type, int size);
        
        virtual ~TaskDescription(){}
        
    private:
        
        
        
    };
    
    typedef std::vector<TaskDescription::Ptr> AggregatedTask;
    typedef std::vector<AggregatedTask> Stack;
    
    
    
    struct CartesianTask : TaskDescription {
        
        std::string base_link, distal_link;
        double orientation_gain;
        
        typedef std::shared_ptr<CartesianTask> Ptr;
        typedef std::shared_ptr<const CartesianTask> ConstPtr;
        
        CartesianTask() = default;
        CartesianTask(std::string distal_link, std::string base_link = "world");
        
        
    };
    
    CartesianTask::Ptr MakeCartesian(std::string distal_link, std::string base_link = "world");
    CartesianTask::Ptr GetAsCartesian(TaskDescription::Ptr task);
    
    
    struct ComTask : TaskDescription {
        
        typedef std::shared_ptr<ComTask> Ptr;
        typedef std::shared_ptr<const ComTask> ConstPtr;
        
        ComTask();
        
    };
    
    ComTask::Ptr MakeCom();
    ComTask::Ptr GetAsCom(TaskDescription::Ptr task);
    
    enum class ConstraintType { JointLimits, VelocityLimits };
    
    struct ConstraintDescription {
        
        ConstraintType type;
        
        typedef std::shared_ptr<ConstraintDescription> Ptr;
        typedef std::shared_ptr<const ConstraintDescription> ConstPtr;
        
        ConstraintDescription() = default;
        ConstraintDescription(ConstraintType type);
        
    };
    
    ConstraintDescription::Ptr MakeJointLimits();
    ConstraintDescription::Ptr MakeVelocityLimits();
        
    class ProblemDescription {
        
    public:
        
        ProblemDescription(TaskDescription::Ptr task);
        ProblemDescription(AggregatedTask task);
        ProblemDescription(Stack stack);
        ProblemDescription(YAML::Node yaml_node);
        
        ProblemDescription& operator<<(ConstraintDescription::Ptr constraint);
        
        int getNumTasks() const;
        AggregatedTask getTask(int id) const;
        
        const std::vector< ConstraintDescription::Ptr >& getBounds() const;
        
    private:
        
        std::vector< std::vector<TaskDescription::Ptr> > _stack;
        std::vector< ConstraintDescription::Ptr > _bounds;
        
    };
    
    
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             TaskDescription::Ptr task_2);
    
    AggregatedTask operator+(AggregatedTask task_1, 
                             TaskDescription::Ptr task_2);
    
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             AggregatedTask task_2);
    
    AggregatedTask operator+(AggregatedTask task_1, 
                             AggregatedTask task_2);
    
    TaskDescription::Ptr operator*(Eigen::Ref<const Eigen::MatrixXd> weight, TaskDescription::Ptr task);
    
    TaskDescription::Ptr operator%(std::vector<int> indices, TaskDescription::Ptr task);
    
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







