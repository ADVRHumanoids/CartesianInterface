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

#include <cartesian_interface/Context.h>
#include <cartesian_interface/problem/Constraint.h>

namespace XBot { namespace Cartesian {
    
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
        * @brief Default (empty) problem
        */
        ProblemDescription() = default;
        
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
         * @brief validate checks if all internal tasks validate() function returns true
         */
        bool validate(bool verbose = false) const;

        TaskDescription::Ptr getTask(const std::string& name);
        
        /**
         * @brief Construct from a YAML description of the IK problem. The required YAML
         * formatting is described at 
         * 
         * https://advrhumanoids.github.io/CartesianInterface/problemdesc.html
         * 
         * @param yaml_node YAML node containing a problem_description node
         * @param model Model of the robot to be controlled (used to retrieve information like the dof number)
         */
        ProblemDescription(YAML::Node yaml_node, Context::ConstPtr context);
        
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

        AggregatedTask getRegularizationTask() const;
        
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
        std::map<std::string, TaskDescription::Ptr> _task_map;
        AggregatedTask _regularisation;
        YAML::Node _solver_options_node;
        
    };
    
    
    
    
} }


#endif







