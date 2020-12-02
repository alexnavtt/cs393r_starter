// This file contains groups of people to test a Social Planner in various scenarios
// Current options:
//   1) TODO: navigating hallway with static people
//   2) TODO: joining conversation between static people
//   3) TODO: avoiding surprise effect with static, unseen person
//   4) TODO: passing by dynamic person in hallway
//   5) TODO: navigating crowded, dynamic space

#ifndef SCENARIOS_CS393R_HH
#define SCENARIOS_CS393R_HH

#include "human.h"

namespace navigation{

struct Scenario{
  int identifier;
  std::string description;
  std::vector<human::Human*> population;
};

}//namespace navigation

#endif