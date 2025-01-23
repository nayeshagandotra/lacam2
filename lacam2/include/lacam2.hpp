#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "utils.hpp"
#include <iostream>

Solution solve(const Instance& ins, std::string& additional_info,
               const int verbose = 0, const Deadline* deadline = nullptr,
               Deadline* opti_deadline = nullptr, bool bool_opti = false, 
               std::string functype = "", std::mt19937* MT = nullptr, 
               const Objective objective = OBJ_NONE, const float restart_rate = 0.001);

