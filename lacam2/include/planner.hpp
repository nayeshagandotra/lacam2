/*
 * lacam-star
 */

#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
std::ostream& operator<<(std::ostream& os, const Objective objective);

// PIBT agent
// struct Agent {
//   const uint id;
//   Vertex* v_now;   // current location
//   Vertex* v_next;  // next location
//   Agent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
// };
// using Agents = std::vector<Agent*>;

struct Agent; 
using Agents = std::vector<Agent*>; 
using Candidates = std::vector<std::array<Vertex*, 5> >;


// PIBT agent
struct Agent {
  int id;
  Vertex* v_now;        // current location
  Vertex* v_next;       // next location
  Vertex* v_next_best;  // best next location
  float tie_breaker;  // epsilon, tie-breaker
  bool is_constrained; //should we recurse through all the actions?
  Agents* group;        // group this belongs to
  int priority;
  // trying new vars
  Candidates C_next;                // next location candidates
  int penalty;
  int action_penalty;
};

// low-level node
struct LNode {
  std::vector<uint> who;
  Vertices where;
  const uint depth;
  LNode(LNode* parent = nullptr, uint i = 0,
        Vertex* v = nullptr);  // who and where
};

// high-level node
struct HNode {
  static uint HNODE_CNT;  // count #(high-level node)
  const Config C;

  // tree
  HNode* parent;
  std::set<HNode*> neighbor;

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)

  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
        const uint _h);
  ~HNode();
};
using HNodes = std::vector<HNode*>;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  Deadline* opti_deadline;
  std::mt19937* MT;
  const int verbose;
  bool opti;
  std::string functype;

  // new ints
  int timestep_penalty;
  int best_penalty;
  int group_no;
  int num_grouped_agents;

  // hyper parameters
  const Objective objective;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  DistTable D;
  uint loop_cnt;      // auxiliary

  // used in PIBT
  std::vector<std::array<Vertex*, 5> > C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;              // random values, used in PIBT
  Agents A;
  Agents A_copy;
  Agents occupied_now;   // for quick collision checking
  Agents occupied_next;  // for quick collision checking

  using Groups = std::vector<Agents*>;
  Groups groups;

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001);
  ~Planner();
  Solution solve(std::string& additional_info);
  void expand_lowlevel_tree(HNode* H, LNode* L);
  void rewrite(HNode* H_from, HNode* T, HNode* H_goal,
               std::stack<HNode*>& OPEN);
  uint get_edge_cost(const Config& C1, const Config& C2);
  uint get_edge_cost(HNode* H_from, HNode* H_to);
  uint get_h_value(const Config& C);
  bool get_new_config(HNode* H, LNode* L);

  // funcpibt versions
  bool funcPIBT(Agent* ai);
  bool funcPIBTwswap(Agent* ai);
  bool funcPIBTgroups(Agent* ai);

  // new functions
  int calculate_penalty(Agent* ai);
  void print_penalty(const std::string& filename, int penalty);
  void refresh_lists(Agents A);
  bool addToGroup(Agent* ai, Agent* aj, bool del_group);
  std::pair<bool, int> OptiPIBT(Agents A, Agent* aj, int accumulated_penalty);


  // swap operation
  Agent* swap_possible_and_required(Agent* ai);
  bool is_swap_required(const uint pusher, const uint puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);

  // utilities
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << HNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};
