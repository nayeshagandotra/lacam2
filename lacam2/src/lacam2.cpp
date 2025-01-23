#include "../include/lacam2.hpp"

Solution solve(const Instance& ins, std::string& additional_info,
               const int verbose, const Deadline* deadline, 
               Deadline* opti_deadline, bool bool_opti, std::string functype, std::mt19937* MT,
               const Objective objective, const float restart_rate)
{
  auto planner = Planner(&ins, deadline, MT, verbose, objective, restart_rate);
  planner.opti_deadline = opti_deadline;
  planner.opti = bool_opti;
  planner.functype = functype;
  std::cout << "opti is " << bool_opti << std::endl;
  std::cout << "deadline is " << opti_deadline->time_limit_ms << std::endl;
  std::cout << "functype is " << planner.functype << std::endl;
  return planner.solve(additional_info);
}
