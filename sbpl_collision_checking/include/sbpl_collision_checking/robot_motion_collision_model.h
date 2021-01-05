////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SBPL_COLLISION_ROBOT_COLLISION_MOTION_MODEL_H
#define SBPL_COLLISION_ROBOT_COLLISION_MOTION_MODEL_H

// standard includes
#include <vector>

// system includes
#include <Eigen/Dense>
#include <smpl/angles.h>
#include <smpl/forward.h>
#include <smpl/types.h>

// project includes
#include <sbpl_collision_checking/robot_collision_model.h>

namespace smpl {
namespace collision {

SBPL_CLASS_FORWARD(RobotMotionCollisionModel);

/// This class is responsible for on-the-fly linear interpolation of motions,
/// given a desired number of evenly-spaced (in joint space) waypoints along the
/// path.
///
/// Queries may be made to generate the n'th waypoint along the path. The
/// application may decide to generate the interpolation samples in any order
class MotionInterpolation
{
public:

    MotionInterpolation(const RobotCollisionModel* rcm);

    int waypointCount() const { return m_waypoint_count; }
    const RobotState& diffs() const { return m_diffs; }

    void setWaypointCount(int waypoint_count);

    void setEndpoints(
        const RobotState& start,
        const RobotState& finish);

    void setEndpoints(
        const RobotState& start,
        const RobotState& finish,
        const std::vector<int>& variables);

    void interpolate(int n, RobotState& state) const;
    void interpolate(int n, RobotState& state, const std::vector<int>& variables) const;

    // TODO: interpolation for a subset of joints

private:

    const RobotCollisionModel* m_rcm;

    RobotState m_start;

    // pre-computed differences from start to finish; for quaternion variables,
    // this contains the final quaternion
    RobotState m_diffs;

    int m_waypoint_count;
    double m_waypoint_count_inv;
};

/// This class is responsible for determining the maximum distance any sphere on
/// the robot might travel for a given motion between two states. It's primary
/// focus is preparing an interpolation of the path such that no sphere moves
/// by more than a given threshold between two waypoints on the path. This
/// threshold can be derived by application safety margins and be used to
/// determine any additional buffers required to guarantee safe motion between
/// the two states. The robot is assumed to follow a linear motion between the
/// initial and final states of the path.
class RobotMotionCollisionModel
{
public:

    RobotMotionCollisionModel(const RobotCollisionModel* rcm);

    const Eigen::Vector3d& motionCenter(int jidx) const;
    double motionRadius(int jidx) const;

    double getMaxSphereMotion(
        const RobotState& start,
        const RobotState& finish) const;

    double getMaxSphereMotion(
        const RobotState& diff) const;

    double getMaxSphereMotion(
        const RobotState& start,
        const RobotState& finish,
        const std::vector<int>& variables) const;

    double getMaxSphereMotion(
        const RobotState& diff,
        const std::vector<int>& variables) const;

    void fillMotionInterpolation(
        const RobotState& start,
        const RobotState& finish,
        double res,
        MotionInterpolation& motion) const;

    void fillMotionInterpolation(
        const RobotState& start,
        const RobotState& finish,
        const std::vector<int>& variables,
        double res,
        MotionInterpolation& motion) const;

private:

    const RobotCollisionModel* m_rcm;

    // spheres bounding the volume that all descendant links can reach; stored
    // per-joint in each joint's frame of reference
    std::vector<Eigen::Vector3d> m_m_centers;
    std::vector<double> m_m_radii;

    // spheres bounding the volume of the immediate child link and the volumes
    // any further descendant links can reach; stored in each joint's frame of
    // reference
    std::vector<Eigen::Vector3d> m_mr_centers;
    std::vector<double> m_mr_radii;
};

inline
MotionInterpolation::MotionInterpolation(const RobotCollisionModel* rcm) :
    m_rcm(rcm),
    m_start(),
    m_diffs(),
    m_waypoint_count(0),
    m_waypoint_count_inv()
{
}

/// Update the waypoint count. A value of 0 or any number greater than 1 is
/// valid. A valid of 1 will be raised to 2 so that interpolation may return
/// the initial and final waypoints.
inline
void MotionInterpolation::setWaypointCount(int waypoint_count)
{
    if (waypoint_count) {
        m_waypoint_count = std::max(2, waypoint_count);
        m_waypoint_count_inv = 1.0 / (double)(m_waypoint_count - 1);
    } else {
        m_waypoint_count = waypoint_count;
    }
}

/// Set the endpoints of the segment to be interpolated. start and finish must
/// contain the same number of variables as the RobotCollisionModel given upon
/// construction.
inline
void MotionInterpolation::setEndpoints(
    const RobotState& start,
    const RobotState& finish)
{
    m_start = start;
    m_diffs.resize(m_rcm->jointVarCount());
    for (size_t jidx = 0; jidx < m_rcm->jointCount(); ++jidx) {
        const int fvidx = m_rcm->jointVarIndexFirst(jidx);
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::REVOLUTE:
        case JointType::PRISMATIC:
            m_diffs[fvidx] = finish[fvidx] - start[fvidx];
            break;
        case JointType::CONTINUOUS:
            m_diffs[fvidx] = angles::shortest_angle_diff(finish[fvidx], start[fvidx]);
            break;
        case JointType::PLANAR:
            m_diffs[fvidx] = finish[fvidx] - start[fvidx];
            m_diffs[fvidx + 1] = finish[fvidx + 1] - start[fvidx + 1];
            m_diffs[fvidx + 2] = angles::shortest_angle_diff(finish[fvidx + 2], start[fvidx + 2]);
            break;
        case JointType::FLOATING:
            m_diffs[fvidx] = finish[fvidx] - start[fvidx];
            m_diffs[fvidx + 1] = finish[fvidx + 1] - start[fvidx + 1];
            m_diffs[fvidx + 2] = finish[fvidx + 2] - start[fvidx + 2];
            m_diffs[fvidx + 3] = finish[fvidx + 3];
            m_diffs[fvidx + 4] = finish[fvidx + 4];
            m_diffs[fvidx + 5] = finish[fvidx + 5];
            m_diffs[fvidx + 6] = finish[fvidx + 6];
            break;
        }
    }
}

inline
void MotionInterpolation::setEndpoints(
    const RobotState& start,
    const RobotState& finish,
    const std::vector<int>& variables)
{
    m_start = start;
    m_diffs.resize(variables.size());
    for (size_t i = 0; i < variables.size(); ++i) {
        int vidx = variables[i];
        int jidx = m_rcm->jointVarJointIndex(vidx);
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::REVOLUTE:
        case JointType::PRISMATIC:
            m_diffs[i] = finish[i] - start[i];
            break;
        case JointType::CONTINUOUS:
            m_diffs[i] = angles::shortest_angle_diff(finish[i], start[i]);
            break;
        case JointType::PLANAR:
            if (m_rcm->jointVarIsContinuous(vidx)) {
                m_diffs[i] = angles::shortest_angle_diff(finish[i], start[i]);
            } else {
                m_diffs[i] = finish[i] - start[i];
            }
            break;
        case JointType::FLOATING:
            ROS_ERROR_ONCE("TODO: interpolation of floating joints for subsets");
            break;
        }
    }
}

/// Interpolate along the segment to generate the n'th waypoint.
inline
void MotionInterpolation::interpolate(int n, RobotState& state) const
{
    state.resize(m_start.size());
    const double alpha = (double)n * m_waypoint_count_inv;
    for (size_t jidx = 0; jidx < m_rcm->jointCount(); ++jidx) {
        const int fvidx = m_rcm->jointVarIndexFirst(jidx);
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::REVOLUTE:
        case JointType::CONTINUOUS:
        case JointType::PRISMATIC:
            state[fvidx] = m_start[fvidx] + alpha * m_diffs[fvidx];
            break;
        case JointType::PLANAR:
            state[fvidx    ] = m_start[fvidx    ] + alpha * m_diffs[fvidx    ];
            state[fvidx + 1] = m_start[fvidx + 1] + alpha * m_diffs[fvidx + 1];
            state[fvidx + 2] = m_start[fvidx + 2] + alpha * m_diffs[fvidx + 2];
            break;
        case JointType::FLOATING:
            state[fvidx    ] = m_start[fvidx    ] + alpha * m_diffs[fvidx    ];
            state[fvidx + 1] = m_start[fvidx + 1] + alpha * m_diffs[fvidx + 1];
            state[fvidx + 2] = m_start[fvidx + 2] + alpha * m_diffs[fvidx + 2];
            const Eigen::Quaterniond q1(
                    m_start[fvidx + 6],
                    m_start[fvidx + 3],
                    m_start[fvidx + 4],
                    m_start[fvidx + 5]);
            const Eigen::Quaterniond q2(
                    m_diffs[fvidx + 6],
                    m_diffs[fvidx + 3],
                    m_diffs[fvidx + 4],
                    m_diffs[fvidx + 5]);
            const Eigen::Quaterniond qi(q1.slerp(alpha, q2));
            state[fvidx + 3] = qi.x();
            state[fvidx + 4] = qi.y();
            state[fvidx + 5] = qi.z();
            state[fvidx + 6] = qi.w();
            break;
        }
    }
}

inline
void MotionInterpolation::interpolate(
    int n,
    RobotState& state,
    const std::vector<int>& variables) const
{
    state.resize(m_start.size());
    const double alpha = (double)n * m_waypoint_count_inv;
    for (size_t v = 0; v < variables.size(); ++v) {
        int vidx = variables[v];
        int jidx = m_rcm->jointVarJointIndex(vidx);
        switch (m_rcm->jointType(jidx)) {
        case JointType::FIXED:
            break;
        case JointType::REVOLUTE:
        case JointType::CONTINUOUS:
        case JointType::PRISMATIC:
        case JointType::PLANAR:
            state[v] = m_start[v] + alpha * m_diffs[v];
            break;
        case JointType::FLOATING:
            ROS_ERROR_ONCE("TODO: interpolation of floating joints for subsets");
            break;
        }
    }
}

inline
const Eigen::Vector3d& RobotMotionCollisionModel::motionCenter(int jidx) const
{
    return m_m_centers[jidx];
}

inline
double RobotMotionCollisionModel::motionRadius(int jidx) const
{
    return m_m_radii[jidx];
}

inline
void RobotMotionCollisionModel::fillMotionInterpolation(
    const RobotState& start,
    const RobotState& finish,
    double res,
    MotionInterpolation& motion) const
{
    motion.setEndpoints(start, finish);
    double max_motion = getMaxSphereMotion(start, finish);
    if (max_motion == 0.0) {
        motion.setWaypointCount(0);
    } else {
        motion.setWaypointCount((int)std::ceil(max_motion / res) + 1);
    }
}

inline
void RobotMotionCollisionModel::fillMotionInterpolation(
    const RobotState& start,
    const RobotState& finish,
    const std::vector<int>& variables,
    double res,
    MotionInterpolation& motion) const
{
    motion.setEndpoints(start, finish, variables);
    double max_motion = getMaxSphereMotion(start, finish, variables);

    if (max_motion == 0.0) {
        motion.setWaypointCount(0);
    } else {
        motion.setWaypointCount((int)std::ceil(max_motion / res) + 1);
    }
}

} // namespace collision
} // namespace smpl

#endif
