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

// project includes
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>

namespace smpl {

static const char* LOG = "heuristic.generic_egraph";

bool GenericEgraphHeuristic::init(RobotPlanningSpace* space, RobotHeuristic* h)
{
    m_checker = space->collisionChecker();
    if (!h) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_orig_h = h;

    m_eg = space->getExtension<ExperienceGraphExtension>();
    if (!m_eg) {
        SMPL_WARN_NAMED(LOG, "GenericEgraphHeuristic recommends ExperienceGraphExtension");
    }

#if 0
    // Floyd Warshall
    ExperienceGraph* eg = m_eg->getExperienceGraph();
    if (!eg) {
        SMPL_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return false;
    }
    auto nodes = eg->nodes();
    dist_matrix.resize(eg->num_nodes(), std::vector<int>(eg->num_nodes()));

    for (size_t i = 0; i < eg->num_nodes(); ++i) {
        for (size_t j = 0; j < eg->num_nodes(); ++j) {
            if (i == j) {
                dist_matrix[i][j] = 0;
            }
            else {
                int h = 0;
                // int h = m_orig_h->GetFromToHeuristic(i_state_id, j_state_id);
                if (eg->edge(i, j)) {
                    // const int edge_cost = 10;
                    dist_matrix[i][j] = h;
                }
                else {
                    // printf("ids %d %d \n", i_state_id, j_state_id);
                    dist_matrix[i][j] = (int)(m_eg_eps * h);
                }
                // printf("node_id: %d %d, dist: %d, h: %d\n",i_nid, j_nid, dist_matrix[i_nid][j_nid], h);
            }
        }
    }

    for (size_t k = 0; k < eg->num_nodes(); ++k) {
        for (size_t i = 0; i < eg->num_nodes(); ++i) {
            for (size_t j = 0; j < eg->num_nodes(); ++j) {
                if (dist_matrix[i][j] > dist_matrix[i][k] + dist_matrix[k][j]) {
                    dist_matrix[i][j] = dist_matrix[i][k] + dist_matrix[k][j];
                }
            }
        }
    }
#endif
    return true;
}

void GenericEgraphHeuristic::setWeightEGraph(double w)
{
    m_eg_eps = w;
    SMPL_DEBUG_NAMED(LOG, "egraph_epsilon: %0.3f", m_eg_eps);
}

void GenericEgraphHeuristic::getEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    ExperienceGraph* eg = m_eg->getExperienceGraph();
    auto nodes = eg->nodes();
    // const int equiv_thresh = 1000;
    const int equiv_thresh = 0;
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        int egraph_state_id = m_eg->getStateID(*nit);
        int h = m_orig_h->GetFromToHeuristic(state_id, egraph_state_id);
        // printf("from %d to %d, h snap %d\n", state_id, egraph_state_id, h);
        if (h <= equiv_thresh) {
            ids.push_back(egraph_state_id);
        }
    }
}

void GenericEgraphHeuristic::getShortcutSuccs(
    int state_id,
    std::vector<int>& shortcut_ids)
{
    if (!m_shortcut_nodes.empty()) {
        // int egraph_state_id = m_eg->getStateID(m_shortcut_nodes[0][0]);
        // shortcut_ids.push_back(egraph_state_id);
        for (size_t i = 0; i < m_shortcut_nodes.size(); ++i) {
            int egraph_state_id = m_eg->getStateID(m_shortcut_nodes[i][0]);
            shortcut_ids.push_back(egraph_state_id);
        }
    }

    // std::vector<ExperienceGraph::node_id> egraph_nodes;
    // m_eg->getExperienceGraphNodes(state_id, egraph_nodes);

    // for (ExperienceGraph::node_id n : egraph_nodes) {
    //     const int comp_id = m_component_ids[n];
    //     for (ExperienceGraph::node_id nn : m_shortcut_nodes[comp_id]) {
    //         int egraph_state_id = m_eg->getStateID(nn);
    //         if (state_id != egraph_state_id) {
    //             shortcut_ids.push_back(egraph_state_id);
    //         }
    //     }
    // }
}

double GenericEgraphHeuristic::getMetricStartDistance(
    double x, double y, double z)
{
    return m_orig_h->getMetricStartDistance(x, y, z);
}

double GenericEgraphHeuristic::getMetricGoalDistance(
    double x, double y, double z)
{
    return m_orig_h->getMetricGoalDistance(x, y, z);
}

Extension* GenericEgraphHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphHeuristicExtension>()) {
        return this;
    }
    return nullptr;
}

void GenericEgraphHeuristic::updateGoal(const GoalConstraint& goal)
{
    m_orig_h->updateGoal(goal);

    if (!m_eg) {
        return;
    }

    ExperienceGraph* eg = m_eg->getExperienceGraph();
    if (!eg) {
        SMPL_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return;
    }

    //////////////////////////////////////////////////////////
    // Compute Connected Components of the Experience Graph //
    //////////////////////////////////////////////////////////

    int comp_count = 0;
    m_component_ids.assign(eg->num_nodes(), -1);
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        if (m_component_ids[*nit] != -1) {
            continue;
        }

        std::vector<ExperienceGraph::node_id> frontier;
        frontier.push_back(*nit);
        while (!frontier.empty()) {
            ExperienceGraph::node_id n = frontier.back();
            frontier.pop_back();

            m_component_ids[n] = comp_count;

            auto adj = eg->adjacent_nodes(n);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                if (m_component_ids[*ait] == -1) {
                    frontier.push_back(*ait);
                }
            }
        }

        ++comp_count;
    }

    SMPL_DEBUG_NAMED(LOG, "Experience graph contains %d connected components", comp_count);


    ////////////////////////////
    // Compute Shortcut Nodes //
    ////////////////////////////

    // NOTE: Everything assumes that the egraph nodes ordering respects the path!!!!
    m_egraph_nodes.clear();

#if 0
    if (comp_count == 2) {
        // SPLIT EGRAPH NODES
        std::vector<int> old_path, new_path;
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            const ExperienceGraph::node_id n = *nit;
            const int comp_id = m_component_ids[n];
            if (comp_id == 0) {
                old_path.push_back(n);
            }
            else {
                new_path.push_back(n);
            }
        }
        printf("old path %zu new path %zu \n", old_path.size(), new_path.size());

        // NEXT STATE ON OLD PATH AFTER NEW START STATE
        size_t start_idx = 0;
        double eps = 1e-6;  // > 0 because of slightly different sampling each time
        for (size_t i = 0; i < old_path.size(); ++i) {
            auto state = eg->state(old_path[i]);
            // printf("i %d, state %f start %f\n", i, state.back(),  m_start.back());
            if (fabs(state.back() - m_start.back()) < eps) {
                start_idx = i;
                break;
            }
        }
        printf("start idx: %zu\n", start_idx);

        // FIRST AND SECOND

        int min_dist = std::numeric_limits<int32_t>::max();
        std::pair<ExperienceGraph::node_id, ExperienceGraph::node_id> closest_pair;
        for (size_t i = start_idx; i < old_path.size(); ++i) {
            if (i != 0) {
                if (!m_checker->isStateToStateValid(eg->state(old_path[i - 1]), eg->state(old_path[i]))) {
                    SMPL_INFO("first path invalid from %f to %f: ",
                        eg->state(old_path[i - 1]).back(), eg->state(old_path[i]).back());
                    break;
                }
            }
            int state_id1 = m_eg->getStateID(old_path[i]);
            for (size_t j = 0; j < new_path.size(); ++j) {
                int state_id2 = m_eg->getStateID(new_path[j]);
                int dist = m_orig_h->GetFromToHeuristic(state_id1, state_id2);
                // printf("i %zu j %zu, dist %d\n", i, j, dist);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_pair.first = old_path[j];
                    closest_pair.second = new_path[j];
                }
            }
        }

        m_shortcut_nodes.assign(comp_count, std::vector<ExperienceGraph::node_id>());
        m_shortcut_nodes[0].push_back(closest_pair.first);

        // THIRD

        int best_h = std::numeric_limits<int32_t>::max();
        for (size_t i = 0; i < new_path.size(); ++i) {
            if (i != 0) {
                if (!m_checker->isStateToStateValid(eg->state(new_path[i - 1]), eg->state(new_path[i]))) {
                    SMPL_INFO("second path invalid from %f to %f: ",
                        eg->state(new_path[i - 1]).back(), eg->state(new_path[i]).back());
                    break;
                }
            }
            int state_id = m_eg->getStateID(new_path[i]);
            int h = m_orig_h->GetGoalHeuristic(state_id);
            if (h < best_h) {
                m_shortcut_nodes[1].clear();
                m_shortcut_nodes[1].push_back(new_path[i]);
            }
            else if (h == best_h) {
                m_shortcut_nodes[1].push_back(new_path[i]);
            }
        }

        // to remove
        m_egraph_nodes.push_back(closest_pair.first);
        m_egraph_nodes.push_back(closest_pair.second);
        m_egraph_nodes.push_back(m_shortcut_nodes[1].front());
        // m_shortcut_nodes[1].push_back(closest_pair.second);

        printf("first %f\n", eg->state(closest_pair.first).back());
        printf("second %f\n", eg->state(closest_pair.second).back());
        printf("third %f\n", eg->state(m_shortcut_nodes[1].front()).back());
    }
#endif

    m_shortcut_nodes.assign(comp_count, std::vector<ExperienceGraph::node_id>());
    std::vector<int> shortcut_heuristics(comp_count);
    int prev_state_id;
    // int comp_id_prev = -1;
    // bool found_sc = false;
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        const ExperienceGraph::node_id n = *nit;
        const int comp_id = m_component_ids[n];
        const int state_id = m_eg->getStateID(n);

        // break as soon as it finds the first invalid edge on the experience
        // if (!found_sc) {
            auto prev_nit = nit;
            ExperienceGraph::node_id prev_n;
            if (nit != nodes.first) {
                prev_n = *prev_nit;
                prev_n--;
                if (nit != nodes.first) {
                    if (!m_checker->isStateToStateValid(eg->state(prev_n), eg->state(n))) {
                        // SMPL_INFO("invalid from %f to %f: ", eg->state(prev_n).back(), eg->state(n).back());
                        break;
                        // found_sc = true;
                    }
                }
            }
        // }
        // if (comp_id_prev != comp_id)
        //     found_sc = false;
        // if (found_sc)
        //     continue;

        int h = m_orig_h->GetGoalHeuristic(state_id);

        if (m_shortcut_nodes[comp_id].empty()) {
            m_shortcut_nodes[comp_id].push_back(n);
            m_egraph_nodes.push_back(n);
            shortcut_heuristics[comp_id] = h;
        } else {
            int best_h = shortcut_heuristics[comp_id];
            auto& s_state = eg->state(n);
            // printf("best_h %d, h %d id %d time %f\n", best_h, h, state_id, s_state.back());
            // if (s_state.back() > 3.5 - 1e-6) {
            if (h < best_h) {
                m_shortcut_nodes[comp_id].clear();
                m_shortcut_nodes[comp_id].push_back(n);
                m_egraph_nodes.clear();
                m_egraph_nodes.push_back(n);
                shortcut_heuristics[comp_id] = h;
            }
            else if (h == best_h) {
                m_shortcut_nodes[comp_id].push_back(n);
                m_egraph_nodes.push_back(n);
            }
            // }
            
        }
    }

    // if (!m_shortcut_nodes.empty()) {
    //     auto state = eg->state(m_shortcut_nodes[0][0]);
    //     SMPL_INFO_STREAM("sc state = " << state);
    // }
    // if (!m_shortcut_nodes.empty())
    //     printf("SC node id %d\n", m_shortcut_nodes[0][0]);
    // printf("num egraph nodes %zu\n", m_egraph_nodes.size());


#if 0
    m_h_nodes.assign(eg->num_nodes() + 1, HeuristicNode(Unknown));
    m_h_nodes[0].dist = 0;
    for (auto i_nit = nodes.first; i_nit != nodes.second; ++i_nit) {
        const ExperienceGraph::node_id i_nid = *i_nit;
        HeuristicNode* i_n = &m_h_nodes[i_nid + 1];
        i_n->dist = std::numeric_limits<int32_t>::max();
        for (auto j_nit = nodes.first; j_nit != nodes.second; ++j_nit) {
            const ExperienceGraph::node_id j_nid = *j_nit;
            const int j_state_id = m_eg->getStateID(j_nid);
            // HeuristicNode* j_n = &m_h_nodes[j_nid + 1];

            // printf("2. id %d\n", j_state_id);
            const int h = m_orig_h->GetGoalHeuristic(j_state_id);
            i_n->dist = std::min(i_n->dist, (int)(m_eg_eps * h) + dist_matrix[i_nid][j_nid]);
        }
        // printf("node: %d H: %d\n", i_nid, i_n->dist);
    }
#endif 

#if 0
    m_h_nodes.assign(eg->num_nodes() + 1, HeuristicNode(Unknown));
    m_open.clear();
    m_h_nodes[0].dist = 0;
    m_open.push(&m_h_nodes[0]);
    while (!m_open.empty()) {
        HeuristicNode* s = m_open.min();
        m_open.pop();

        int nidx = std::distance(m_h_nodes.data(), s);
        if (nidx == 0) {
            // neighbors: inflated edges to all experience graph states
            // unconditionally relaxed (goal node is the first node removed)
            for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                const ExperienceGraph::node_id nid = *nit;
                HeuristicNode* n = &m_h_nodes[nid + 1];
                const int state_id = m_eg->getStateID(nid);
                const int h = m_orig_h->GetGoalHeuristic(state_id);
                n->dist = (int)(m_eg_eps * h);
                m_open.push(n);
            }
        } else {
            // neighbors: inflated edges to all non-adjacent experience graph
            // states original cost edges to all adjacent experience graph
            // states
            const ExperienceGraph::node_id sid = nidx - 1;
            const int s_state_id = m_eg->getStateID(sid);
            for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                const ExperienceGraph::node_id nid = *nit;
                HeuristicNode* n = &m_h_nodes[nid + 1];
                if (eg->edge(sid, nid)) {
                    const int edge_cost = 10;
                    const int new_cost = s->dist + edge_cost;
                    if (new_cost < n->dist) {
                        n->dist = new_cost;
                        if (m_open.contains(n)) {
                            m_open.decrease(n);
                        } else {
                            m_open.push(n);
                        }
                    }
                } else {
                    const int n_state_id = m_eg->getStateID(nid);
                    int h = m_orig_h->GetFromToHeuristic(s_state_id, n_state_id);
                    const int new_cost = s->dist + (int)(m_eg_eps * h);
                    if (new_cost < n->dist) {
                        n->dist = new_cost;
                        if (m_open.contains(n)) {
                            m_open.decrease(n);
                        } else {
                            m_open.push(n);
                        }
                    }
                }
            }
        }
    }
#endif
}

// void GenericEgraphHeuristic::updateStart(const RobotState start)
// {
//     m_start = start;
//     printf("updated start state time: %f\n", start.back());
// }

bool GenericEgraphHeuristic::isReplanCutoffBeforeShortcutNode(double replan_cutoff)
{
    if (m_shortcut_nodes.size() == 0) {
        return true;
    }
    ExperienceGraph* eg = m_eg->getExperienceGraph();
    auto state = eg->state(m_shortcut_nodes[0][0]);
    // if (replan_cutoff > state.back()) {
    //     printf("PROBLEM %.3f\n", state.back());
    // }
    return replan_cutoff <= state.back();
}


// int GenericEgraphHeuristic::GetGoalHeuristic(int state_id)
// {
//     if (!m_eg) {
//         return 0;
//     }

//     ExperienceGraph* eg = m_eg->getExperienceGraph();
//     if (!eg) {
//         return 0;
//     }

//     int best_h = (int)(m_eg_eps * m_orig_h->GetGoalHeuristic(state_id));
//     // return best_h;
//     // printf("id: %d h before %d\n", state_id, best_h);
//     // auto nodes = eg->nodes();
//     // for (auto nit = nodes.first; nit != nodes.second; ++nit) {
//     // auto nodes = m_shortcut_nodes;
//     for (size_t i = 0; i < m_shortcut_nodes.size(); ++i) {  // i is comp id
//         for (ExperienceGraph::node_id nn : m_shortcut_nodes[i]) {
//             const int egraph_state_id = m_eg->getStateID(nn);
//             const int h = m_orig_h->GetFromToHeuristic(state_id, egraph_state_id);
//             const int dist = m_h_nodes[nn + 1].dist;
//             const int new_h = dist + (int)(m_eg_eps * h);
//             // printf("    best %d new %d dist %d h %d\n", best_h, new_h, dist, h);
//             if (new_h < best_h) {
//                 best_h = new_h;
//             }
//         }
//     }

//     // printf("id: %d h after %d\n", state_id, best_h);
//     // getchar();

//     return best_h;
// }

int GenericEgraphHeuristic::GetGoalHeuristic(int state_id)
{
    if (!m_eg) {
        return 0;
    }

    ExperienceGraph* eg = m_eg->getExperienceGraph();
    if (!eg) {
        return 0;
    }

    int best_h = (int)(m_eg_eps * m_orig_h->GetGoalHeuristic(state_id));
    // auto nodes = eg->nodes();
    // for (ExperienceGraph::node_id nid : m_egraph_nodes) {
    //     const int egraph_state_id = m_eg->getStateID(nid);
    //     const int h = m_orig_h->GetFromToHeuristic(state_id, egraph_state_id);
    //     const int dist = m_h_nodes[nid + 1].dist;
    //     const int new_h = dist + (int)(m_eg_eps * h);
    //     if (new_h < best_h) {
    //         best_h = new_h;
    //     }
    // }

    return best_h;
}

int GenericEgraphHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int GenericEgraphHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

} // namespace smpl
