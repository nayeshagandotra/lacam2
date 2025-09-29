#include "../include/planner.hpp"

LNode::LNode(LNode* parent, uint i, Vertex* v)
    : who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1)
{
  if (parent != nullptr) {
    who = parent->who;
    who.push_back(i);
    where = parent->where;
    where.push_back(v);
  }
}

uint HNode::HNODE_CNT = 0;

// for high-level
HNode::HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
             const uint _h)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<LNode*>())
{
  ++HNODE_CNT;

  search_tree.push(new LNode());
  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) parent->neighbor.insert(this);

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (uint i = 0; i < N; ++i) priorities[i] = (float)D.get(i, C[i]) / N;
  } else {
    // dynamic priorities, akin to PIBT
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](uint i, uint j) { return priorities[i] > priorities[j]; });
}

HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const Objective _objective, const float _restart_rate)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      RESTART_RATE(_restart_rate),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr)
{
}

Planner::~Planner() {}

int Planner::calculate_penalty(Agent* ai) {
  
  const auto i = ai->id;
  // calculate ideal dist for penalty purposes
  int ideal_dist = D.get(ai->id, ai->C_next[i][0]);  // Distance to goal if taking ideal move
  int actual_dist = D.get(ai->id, ai->v_next); // Distance to goal based on suggested move
  return actual_dist - ideal_dist;
}

void Planner::print_penalty(const std::string& filename, int penalty) {
    // std::string output_dir = "code/output/";
    std::string full_filename = filename;
    
    std::ofstream outFile(full_filename, std::ios::app);  // Open in append mode
    if (!outFile) {
        std::cerr << "Error opening file: " << full_filename << "\n";
        return;
    }

    // Write start positions
    outFile << penalty << "\n";
    outFile.close();
}

void Planner::refresh_lists(Agents A){
  // clear occupied next (local list) before next optipibt iter
  for (auto a : A) {
    // clear
    if (a->v_next != nullptr){
      if (occupied_next[a->v_next->id] == a){
        occupied_next[a->v_next->id] = nullptr;
      }
      a->v_next = nullptr;
    }
  }
}

bool Planner::addToGroup(Agent* ai, Agent* aj, bool del_group) {
  // if not in opti mode,
  // if any agent is constrained, don't add to group
  if (!opti || ai->is_constrained || aj->is_constrained) {
    return false;
  }

  Agents* ng;

  if (ai->group != nullptr && aj->group == nullptr) {
    ng = ai->group;
    // add aj to group- assume ai is already in g
    ng->push_back(aj);
    aj->group = ng;
    num_grouped_agents += 1;
    return true;

  } else if (aj->group != nullptr && ai->group == nullptr) {
    ng = aj->group;
    ng->push_back(ai);
    ai->group = ng;
    num_grouped_agents += 1;
    return true;

  } else if (aj->group == nullptr && ai->group == nullptr) {
    // neither have group
    ng = new Agents();
    ng->push_back(ai);
    ng->push_back(aj);
    ai->group = ng;
    aj->group = ng;
    groups.push_back(ng);
    num_grouped_agents += 2;

  } else if ((aj->group != nullptr && ai->group != nullptr) && aj->group != ai->group) {
    // merge groups
    Agents* group1 = ai->group;
    Agents* group2 = aj->group;

    // Create a new group that contains all agents from both groups
    Agents* new_group = new Agents();
    std::set<Agent*> unique_agents;

    // Insert all agents from group1 into the set
    for (const auto& agent : *group1) {
      unique_agents.insert(agent);
    }

    // Insert all agents from group2 into the set, avoiding duplicates
    for (const auto& agent : *group2) {
      unique_agents.insert(agent);
    }

    // Reserve space in new_group to avoid unnecessary reallocations
    new_group->reserve(unique_agents.size());

    // Copy unique agents back to new_group
    for (const auto& agent : unique_agents) {
      new_group->push_back(agent);
    }

    // Update group pointers for all agents in both groups
    for (auto& agent : *group1) {
      agent->group = new_group;
    }
    for (auto& agent : *group2) {
      agent->group = new_group;
    }

    // Add the new group to the groups list
    groups.push_back(new_group);

    // If del_group is true, delete redundant groups
    if (del_group) {
      groups.erase(std::remove(groups.begin(), groups.end(), group1), groups.end());
      groups.erase(std::remove(groups.begin(), groups.end(), group2), groups.end());

      // Free the memory for group1 and group2
      delete group1;
      delete group2;
    }
  }
  return false;
}

std::pair<bool, int> Planner::OptiPIBT(Agents A, Agent* aj, int accumulated_penalty){

  std::sort(A.begin(), A.end(), [](Agent* a, Agent* b) {
        return a->priority < b->priority; // Ascending order
    });
  Agent* ai;
  // pick highest priority agent 
  if (aj == nullptr){
    ai = A[0];
  }
  else{
    ai = aj;
  }

  if (is_expired_ns(opti_deadline)){
    return std::make_pair(true, 100000);
  }

  // get subset of agents
  Agents agents_subset;
  // Copy all pointers except for agent ai
  std::copy_if(A.begin(), A.end(), std::back_inserter(agents_subset), 
                [ai](Agent* agent) { return agent != ai; });  

  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // // get candidates for next locations
  // for (size_t k = 0; k < K; ++k) {
  //   auto u = ai->v_now->neighbor[k];
  //   C_next[i][k] = u;
  //   if (MT != nullptr)
  //     tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker get_random_float(MT)
  // }
  // C_next[i][K] = ai->v_now;

  // // sort, note: K + 1 is sufficient
  // std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
  //           [&](Vertex* const v, Vertex* const u) {
  //             return D.get(i, v) + tie_breakers[v->id] <
  //                    D.get(i, u) + tie_breakers[u->id];
  //           });

  // calculate ideal dist for penalty purposes
  int ideal_dist = D.get(ai->id, ai->C_next[i][0]);  // Distance to goal if taking ideal move
  int actual_dist;
  int round_bestp = 100000;
  bool group_exists = false;

  // counting number of skipped actions
  int n_avail_acts = 0;
  int n_skipped_acts = 0;

  
  for (size_t k = 0; k < K + 1; ++k) {
    auto u = ai->C_next[i][k];   //include best action for completeness
    refresh_lists(A); // clear the results from the last PIBT call
    n_avail_acts += 1;
    actual_dist = D.get(ai->id, u);
    int action_penalty =  (actual_dist - ideal_dist); //0 if equal
    if (action_penalty + accumulated_penalty >= best_penalty){
      n_skipped_acts += 1;
      continue; //bad action
    }
    if (action_penalty > ai->penalty){
      n_skipped_acts += 1;
      continue; //bad action because pibt found better
    }
    if (occupied_next[u->id] != nullptr && occupied_next[u->id] != ai){
      // The agent at vertex 'id' in occupied_next is not nullptr and not in A
      // and it's not the current agent
      // means it's been reserved by a previous fixed agent or by a constraint
      group_exists = false;
      for (size_t i = group_no + 1; i < groups.size(); ++i) {
        if (groups[i] == ai->group) {
            group_exists = true;
            break;
        }
      }
      if (addToGroup(occupied_next[u->id], ai, false) && !group_exists){
        // need to add this new group to the groups list because something new has been added
        groups.push_back(ai->group);
      }
      n_skipped_acts += 1;
      continue;
    }

    // if action is causing swap conflict, continue
    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak != ai){
      // The agent at vertex 'id' in occupied_now is not nullptr and not in A
      // and it's not the current agent
      // means it's been reserved by a previous fixed agent
      // // check if we have a swap conflict
      if (ak->v_next == ai->v_now){
        group_exists = false;
        for (size_t i = group_no + 1; i < groups.size(); ++i) {
            if (groups[i] == ai->group) {
                group_exists = true;
                break;
            }
        }
        if (addToGroup(ak, ai, false) && !group_exists){
          // need to add this new group to the groups list because something new has been added
          groups.push_back(ai->group);
        }
        // the node we are trying to move to is currently occupied by an agent that 
        // wants to move to us or stay at u. since we are lower priority, we sacrifice this action.
        n_skipped_acts += 1;
        continue;
      }
    }
    // option 2: try recursing through ak and skip action if it doesn't improve
    occupied_next[u->id] = ai;
    ai->v_next = u; 
    ai->action_penalty = action_penalty;
    // there is an unplanned agent here, let's see if we can comfortably move it
    if (ak != nullptr && ak->v_next == nullptr){
      group_exists = false;
      for (size_t i = group_no + 1; i < groups.size(); ++i) {
        if (groups[i] == ai->group) {
            group_exists = true;
            break;
        }
      }
      if (addToGroup(ak, ai, false) && !group_exists){
        // need to add this new group to the groups list because something new has been added
        groups.push_back(ai->group);
      }
      auto [failed, bas] = OptiPIBT(agents_subset, ak, accumulated_penalty + action_penalty);
      if (failed){
        n_skipped_acts += 1;
        // we tried moving to this action and moving other agents accordingly, but agent ak is stuck
        // ai->v_next = nullptr;
        continue;
      }
      round_bestp = std::min(round_bestp, action_penalty + bas);
      continue; // regardless, try another action now, this one has been explored already (if it's best, itll get saved at the bottom)
    }
   
    
    // run OptiPIBT recursive call if not last agent
    int best_after_subset = 0;
    bool f = true;
    if (!agents_subset.empty()) {
      // agents_subset is not empty
      std::pair<bool, int> result = OptiPIBT(agents_subset, nullptr, accumulated_penalty + action_penalty);
      if (result.first) continue;
      best_after_subset = result.second;
    } 
    round_bestp = std::min(round_bestp, action_penalty + best_after_subset);
    // this should give it some new v_next values
    if (accumulated_penalty + action_penalty + best_after_subset < best_penalty){
      best_penalty = accumulated_penalty + action_penalty + best_after_subset;
      for (auto agent : A_copy) {
        // Set the agent's v_next_best to v_next (which should be correct atm)
        agent->v_next_best = agent->v_next; 
        agent->penalty = agent->action_penalty;
      }
      if (best_penalty == 0){
        return std::make_pair(false, round_bestp);
      }
    }
  }
  // either all moves have failed or next best has been found
  if (n_skipped_acts == n_avail_acts){
    // failed to secure node
    // occupied_next[ai->v_now->id] = ai;
    // ai->v_next = ai->v_now;
    return std::make_pair(true, 100000);
  }
  return std::make_pair(false, round_bestp);
}


Solution Planner::solve(std::string& additional_info)
{
  solver_info(1, "start search");

  // setup agents
  // for (auto i = 0; i < N; ++i) A[i] = new Agent(i);
  // setup agents
  for (auto i = 0; i < N; ++i){
    Agent* a = new Agent{i,                          // id
                         nullptr,                    // current location
                         nullptr,                    // next location (local)
                         nullptr,                    // next best location
                         get_random_float(MT)};      // tie-breaker
                         false;                      // constraints remembrance
    A[i] = a;
  } 

  // setup search
  auto OPEN = std::stack<HNode*>();
  auto EXPLORED = std::unordered_map<Config, HNode*, ConfigHasher>();
  // insert initial node, 'H': high-level node
  auto H_init = new HNode(ins->starts, D, nullptr, 0, get_h_value(ins->starts));
  OPEN.push(H_init);
  EXPLORED[H_init->C] = H_init;

  std::vector<Config> solution;
  auto C_new = Config(N, nullptr);  // for new configuration
  HNode* H_goal = nullptr;          // to store goal node

  // DFS
  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    auto H = OPEN.top();  // high-level node

    // low-level search end
    if (H->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // check lower bounds
    if (H_goal != nullptr && H->f >= H_goal->f) {
      OPEN.pop();
      continue;
    }

    // check goal condition
    if (H_goal == nullptr && is_same_config(H->C, ins->goals)) {
      H_goal = H;
      solver_info(1, "found solution, cost: ", H->g);
      if (objective == OBJ_NONE) break;
      continue;
    }

    // create successors at the low-level search
    auto L = H->search_tree.front();
    H->search_tree.pop();
    expand_lowlevel_tree(H, L);

    // create successors at the high-level search
    const auto res = get_new_config(H, L);
    delete L;  // free
    if (!res) continue;

    // create new configuration
    for (auto a : A) C_new[a->id] = a->v_next;

    // check explored list
    const auto iter = EXPLORED.find(C_new);
    if (iter != EXPLORED.end()) {
      // case found
      rewrite(H, iter->second, H_goal, OPEN);
      // re-insert or random-restart
      auto H_insert = (MT != nullptr && get_random_float(MT) >= RESTART_RATE)
                          ? iter->second
                          : H_init;
      if (H_goal == nullptr || H_insert->f < H_goal->f) OPEN.push(H_insert);
    } else {
      // insert new search node
      const auto H_new = new HNode(
          C_new, D, H, H->g + get_edge_cost(H->C, C_new), get_h_value(C_new));
      EXPLORED[H_new->C] = H_new;
      if (H_goal == nullptr || H_new->f < H_goal->f) OPEN.push(H_new);
    }
  }

  // backtrack
  if (H_goal != nullptr) {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->C);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  // print result
  if (H_goal != nullptr && OPEN.empty()) {
    solver_info(1, "solved optimally, objective: ", objective);
  } else if (H_goal != nullptr) {
    solver_info(1, "solved sub-optimally, objective: ", objective);
  } else if (OPEN.empty()) {
    solver_info(1, "no solution");
  } else {
    solver_info(1, "timeout");
  }

  // logging
  additional_info +=
      "optimal=" + std::to_string(H_goal != nullptr && OPEN.empty()) + "\n";
  additional_info += "objective=" + std::to_string(objective) + "\n";
  additional_info += "loop_cnt=" + std::to_string(loop_cnt) + "\n";
  additional_info += "num_node_gen=" + std::to_string(EXPLORED.size()) + "\n";

  // memory management
  for (auto a : A) delete a;
  for (auto itr : EXPLORED) delete itr.second;

  return solution;
}

void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal,
                      std::stack<HNode*>& OPEN)
{
  // update neighbors
  H_from->neighbor.insert(H_to);

  // Dijkstra update
  std::queue<HNode*> Q({H_from});  // queue is sufficient
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
      if (g_val < n_to->g) {
        if (n_to == H_goal)
          solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
      }
    }
  }
}

uint Planner::get_edge_cost(const Config& C1, const Config& C2)
{
  if (objective == OBJ_SUM_OF_LOSS) {
    uint cost = 0;
    for (uint i = 0; i < N; ++i) {
      if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
        cost += 1;
      }
    }
    return cost;
  }

  // default: makespan
  return 1;
}

uint Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
  return get_edge_cost(H_from->C, H_to->C);
}

uint Planner::get_h_value(const Config& C)
{
  uint cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (auto i = 0; i < N; ++i) cost = std::max(cost, D.get(i, C[i]));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (auto i = 0; i < N; ++i) cost += D.get(i, C[i]);
  }
  return cost;
}

void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
{
  if (L->depth >= N) return;
  const auto i = H->order[L->depth];
  auto C = H->C[i]->neighbor;
  C.push_back(H->C[i]);
  // randomize
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  // insert
  for (auto v : C) H->search_tree.push(new LNode(L, i, v));
}

bool Planner::get_new_config(HNode* H, LNode* L)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }
    if (a->v_next_best != nullptr) {
      // occupied_next[a->v_next_best->id] = nullptr;
      a->v_next_best = nullptr;
    }
    a->group = nullptr;
    a->is_constrained = false;

    // set occupied now
    a->v_now = H->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (uint k = 0; k < L->depth; ++k) {
    const auto i = L->who[k];        // agent
    const auto l = L->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto l_pre = H->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    // set occupied_next
    A[i]->v_next = L->where[k];
    occupied_next[l] = A[i];
    A[i]->is_constrained = true;
  }

  // perform PIBT
  timestep_penalty = 0;
  int pi = 0;
  num_grouped_agents = 0;
  for (auto k : H->order) {
    auto a = A[k];
    a->priority = pi;
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
    pi ++;
  }
  // if opti, refine with opti-pibt
  if (opti && timestep_penalty != 0) {
    group_no = 0;
    opti_deadline->reset();

    double overall_deadline = opti_deadline->time_limit_ms;

    //   && 
    for (size_t i = 0; i < groups.size(); ++i) {

      if (is_expired(opti_deadline)) break;

      // std::cout << "i = " << i << std::endl; 
      group_no = i;
      Agents* g = groups[i];
      A_copy = *g;
      best_penalty = 100000;
      int num_agents = 0;
      int tsp = 0;

      for (auto a : A_copy) {
        tsp+= a->penalty;
        num_agents += 1;
        a->v_next_best = a->v_next; // Reserve PIBT answer for cutoff reasons
      }

      if (tsp == 0){
        i++;
        continue;
      }

      opti_deadline->time_limit_ns = overall_deadline * 1000000; // Convert ms to ns
      opti_deadline->time_limit_ns *= static_cast<double>(num_agents) / num_grouped_agents;

      auto start = std::chrono::high_resolution_clock::now();
      auto [failed, bas] = OptiPIBT(A_copy, nullptr, 0);
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

      bool group_exists = false;
      for (size_t i = group_no + 1; i < groups.size(); ++i) {
        if (groups[i] == A_copy[0]->group) {
            group_exists = true;
            break;
        }
      }  
      if (failed && !group_exists){
        // need to add this new group to the groups list because something new has been added
        groups.push_back(groups[group_no]);
      }

      overall_deadline -= duration;

      refresh_lists(A_copy);

      // If new groups are added, they will be processed in the next iteration
      for (auto a : A_copy) {
          occupied_next[a->v_next_best->id] = a; // Reserve
          a->v_next = a->v_next_best;
      }
    }
  }
  // this is so in the very end
  groups.clear();
  return true;
}
bool Planner::funcPIBT(Agent* ai) {
  return funcPIBTgroups(ai);
    // if (functype == "opti") {
    //     // vanilla pibt with grouping logic
        
    // } else {
    //   return funcPIBTwswap(ai);
    // }
}

bool Planner::funcPIBTwswap(Agent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });
  ai->C_next = C_next;

  Agent* swap_agent = swap_possible_and_required(ai);
  if (swap_agent != nullptr)
    std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);

  // main operation
  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // priority inheritance
    if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBTwswap(ak))
      continue;

    // success to plan next one step
    // pull swap_agent when applicable
    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

bool Planner::funcPIBTgroups(Agent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (size_t k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort, note: K + 1 is sufficient
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });

  ai->C_next = C_next;
  ai->penalty = 0;
  // calculate ideal dist for penalty purposes
  int ideal_dist = D.get(ai->id, C_next[i][0]);  // Distance to goal if taking ideal move
  int actual_dist;
  int diff;

  for (size_t k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
     if (occupied_next[u->id] != nullptr){
      addToGroup(ai, occupied_next[u->id], true);
      continue;
    } 

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts with constraints (and with inherited agents)
    // this condition takes care of aj swap
    if (ak != nullptr && ak->v_next == ai->v_now){
      addToGroup(ai, ak, true);
      continue;
    } 

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // empty or stay
    if (ak == nullptr || u == ai->v_now) return true;  //why does this second condition exist?

    // priority inheritance
    if (ak->v_next == nullptr){
      addToGroup(ai, ak, true);
      if (!funcPIBTgroups(ak)) continue;
    } 

    // PENALTY FOR LOGGING PURPOSES
    actual_dist = D.get(ai->id, ai->v_next);
    diff = actual_dist - ideal_dist;
    timestep_penalty += diff; //0 if equal
    ai->penalty = diff;

    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;

  // find penalty
  actual_dist = D.get(ai->id, ai->v_next);
  diff = actual_dist - ideal_dist;
  timestep_penalty += diff; //0 if equal
  ai->penalty = diff;
  return false;
}


Agent* Planner::swap_possible_and_required(Agent* ai)
{
  const auto i = ai->id;
  // ai wanna stay at v_now -> no need to swap
  if (C_next[i][0] == ai->v_now) return nullptr;

  // usual swap situation, c.f., case-a, b
  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
      is_swap_possible(aj->v_now, ai->v_now)) {
    return aj;
  }

  // for clear operation, c.f., case-c
  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_swap_possible(C_next[i][0], ai->v_now)) {
      return ak;
    }
  }

  return nullptr;
}

// simulate whether the swap is required
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (D.get(pusher, v_puller) < D.get(pusher, v_pusher)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (D.get(puller, v_pusher) < D.get(puller, v_puller)) &&
         (D.get(pusher, v_pusher) == 0 ||
          D.get(pusher, v_puller) < D.get(pusher, v_pusher));
}

// simulate whether the swap is possible
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();  // count #(possible locations) to pull
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;      // pull-impossible with u
      } else {
        tmp = u;  // pull-possible with u
      }
    }
    if (n >= 2) return true;  // able to swap
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

std::ostream& operator<<(std::ostream& os, const Objective obj)
{
  if (obj == OBJ_NONE) {
    os << "none";
  } else if (obj == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (obj == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}
