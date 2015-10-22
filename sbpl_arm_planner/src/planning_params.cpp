/*
 * Copyright (c) 2010, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 /** \author Benjamin Cohen */

#include <sbpl_arm_planner/planning_params.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>
 
namespace sbpl_arm_planner {

const bool PlanningParams::DefaultSearchMode = false;
const bool PlanningParams::DefaultShortcutPath = false;
const bool PlanningParams::DefaultInterpolatePath = false;
const bool PlanningParams::DefaultUseMultipleIkSolutions = false;
const double PlanningParams::DefaultAllowedTime = 10.0;
const double PlanningParams::DefaultWaypointTime = 0.35;
const bool PlanningParams::DefaultUseBfsHeuristic = true;
const double PlanningParams::DefaultEpsilon = 10.0;
const double PlanningParams::DefaultPlanningLinkSphereRadius = 0.08;
const int PlanningParams::DefaultCostMultiplier = 1000;
const int PlanningParams::DefaultCostPerCell = 1;
const int PlanningParams::DefaultCostPerMeter = 50;
const int PlanningParams::DefaultCostPerSecond = DefaultCostMultiplier;
const double PlanningParams::DefaultTimePerCell = 0.05;
const double PlanningParams::DefaultMaxMprimOffset = 0.0;
const std::string PlanningParams::DefaultExpandsLog = "info";
const std::string PlanningParams::DefaultExpands2Log = "info";
const std::string PlanningParams::DefaultIkLog = "info";
const std::string PlanningParams::DefaultRobotModelLog = "info";
const std::string PlanningParams::DefaultCspaceLog = "info";
const std::string PlanningParams::DefaultSolutionLog = "info";

PlanningParams::PlanningParams() :
    search_mode_(DefaultSearchMode),
    shortcut_path_(DefaultShortcutPath),
    interpolate_path_(DefaultInterpolatePath),
    use_multiple_ik_solutions_(DefaultUseMultipleIkSolutions),
    allowed_time_(DefaultAllowedTime),
    waypoint_time_(DefaultWaypointTime),

    ready_to_plan_(false),
    num_joints_(0),
    planning_frame_(),
    group_name_(),
    planner_name_(),
    planning_joints_(),

    use_bfs_heuristic_(DefaultUseBfsHeuristic),
    epsilon_(DefaultEpsilon),
    planning_link_sphere_radius_(DefaultPlanningLinkSphereRadius),

    coord_vals_(),
    coord_delta_(),

    cost_multiplier_(DefaultCostMultiplier),
    cost_per_cell_(DefaultCostPerCell),
    cost_per_meter_(DefaultCostPerMeter),
    cost_per_second_(DefaultCostPerSecond),
    time_per_cell_(DefaultTimePerCell),
    max_mprim_offset_(DefaultMaxMprimOffset),

    print_path_(true),
    verbose_(false),
    verbose_heuristics_(false),
    verbose_collisions_(false),
    expands_log_("expands"),
    expands2_log_("expands2"),
    ik_log_("ik"),
    rmodel_log_("arm"),
    cspace_log_("cspace"),
    solution_log_("solution"),
    expands_log_level_(DefaultExpandsLog),
    expands2_log_level_(DefaultExpands2Log),
    ik_log_level_(DefaultIkLog),
    rmodel_log_level_(DefaultRobotModelLog),
    cspace_log_level_(DefaultCspaceLog),
    solution_log_level_(DefaultSolutionLog)
{
}

bool PlanningParams::init(const std::string& ns)
{
    ros::NodeHandle nh(ns);
    ROS_ERROR("Getting params from namespace: %s", nh.getNamespace().c_str());
    /* planning */
    nh.param("planning/epsilon", epsilon_, 10.0);
    nh.param("planning/use_bfs_heuristic", use_bfs_heuristic_, true);
    nh.param("planning/verbose", verbose_, false);
    nh.param("planning/verbose_collisions", verbose_collisions_, false);
    nh.param ("planning/search_mode", search_mode_, false); //true: stop after first solution
    nh.param ("planning/shortcut_path", shortcut_path_, false);
    nh.param ("planning/interpolate_path", interpolate_path_, false);
    nh.param ("planning/use_multiple_ik_solutions", use_multiple_ik_solutions_, false);
    nh.param ("planning/seconds_per_waypoint", waypoint_time_, 0.35);
    nh.param<std::string>("planning/planning_frame", planning_frame_, "");
    nh.param<std::string>("planning/group_name", group_name_, "");
    
    /* logging */
    nh.param ("debug/print_out_path", print_path_, true);
    nh.param<std::string>("debug/logging/expands", expands_log_level_, "info");
    nh.param<std::string>("debug/logging/expands2", expands2_log_level_, "info");
    nh.param<std::string>("debug/logging/ik", ik_log_level_, "info");
    nh.param<std::string>("debug/logging/robot_model", rmodel_log_level_, "info");
    nh.param<std::string>("/debug/logging/collisions", cspace_log_level_, "info");
    nh.param<std::string>("debug/logging/solution", solution_log_level_, "info");
    
    /* planning joints */
    XmlRpc::XmlRpcValue xlist;
    nh.getParam("planning/planning_joints", xlist);
    std::string joint_list = std::string(xlist);
    std::stringstream joint_name_stream(joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.size() == 0) {
            continue;
        }
        planning_joints_.push_back(jname);
    }
    num_joints_ = planning_joints_.size();
    
    // discretization
    std::string p;
    if (nh.hasParam("planning/discretization")) {
        nh.getParam("planning/discretization", xlist);
        std::stringstream ss(xlist);
        while (ss >> p) {
            coord_vals_.push_back(atof(p.c_str()));
        }
    
        coord_delta_.resize(coord_vals_.size(), 0);
        for (int i = 0; i < num_joints_; ++i) {
            coord_delta_[i] = (2.0 * M_PI) / coord_vals_[i];
        }
    }
    else {
        ROS_ERROR("Discretization of statespace has not been defined.");
        return false;
    }
    
    leatherman::setLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands_log_, expands_log_level_);
    leatherman::setLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + expands2_log_, expands2_log_level_);
    leatherman::setLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + solution_log_, solution_log_level_);
    leatherman::setLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + ik_log_, ik_log_level_);
    leatherman::setLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + rmodel_log_, rmodel_log_level_);
    leatherman::setLoggerLevel(ROSCONSOLE_DEFAULT_NAME + std::string(".") + cspace_log_, cspace_log_level_);
    
    return true;
}

void PlanningParams::printParams(const std::string& stream) const
{
    ROS_INFO_NAMED(stream, " ");
    ROS_INFO_NAMED(stream, "Manipulation Environment Parameters:");
    ROS_INFO_NAMED(stream, "%40s: %.2f", "epsilon",epsilon_);
    ROS_INFO_NAMED(stream, "%40s: %s", "use dijkstra heuristic", use_bfs_heuristic_ ? "yes" : "no");
    ROS_INFO_NAMED(stream, "%40s: %s", "sbpl search mode", search_mode_ ? "stop_after_first_sol" : "run_until_timeout");
    ROS_INFO_NAMED(stream, "%40s: %s", "postprocessing: shortcut", shortcut_path_ ? "yes" : "no");
    ROS_INFO_NAMED(stream, "%40s: %s", "postprocessing: interpolate", interpolate_path_ ? "yes" : "no");
    ROS_INFO_NAMED(stream, "%40s: %0.3fsec", "time_per_waypoint", waypoint_time_);
    ROS_INFO_NAMED(stream, "%40s: %d", "cost per cell", cost_per_cell_);
    ROS_INFO_NAMED(stream, "%40s: %s", "reference frame", planning_frame_.c_str());
    ROS_INFO_NAMED(stream, "%40s: %s", "group name", group_name_.c_str());
    ROS_INFO_NAMED(stream, "planning joints: ");
    for (size_t i = 0; i < planning_joints_.size(); ++i) {
        ROS_INFO_NAMED(stream, "   [%d] %30s", int(i), planning_joints_[i].c_str());
    }
    ROS_INFO_NAMED(stream, "discretization: ");
    for (size_t i = 0; i < coord_vals_.size(); ++i) {
        ROS_INFO_NAMED(stream, "   [%d] val: %d  delta: %0.3f", int(i), coord_vals_[i], coord_delta_[i]);
    }
    ROS_INFO_NAMED(stream, " ");
}

} // namespace sbpl_arm_planner
