#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>
#include <map>
#include <string>

// FSM States
#include "XFsmState.h"

using namespace std;

class XFsmMgr {
 public:
  XFsmMgr();

  // Initializes the Control FSM instance
  void initialize();

  void addState(XFsmState* state);
  void addState(XFsmState* state, int firstflag);
  XFsmState* getStateByName(string name);

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM(string& event);
  // return currentstate name
  void currentstatename(string& state);

  // FSM state information
  std::map<string, XFsmState* > XFsmStatesMap;

 private:
  string firstStateName;
  XFsmState* currentState;
  XFsmState* nextState;

};

#endif  // CONTROLFSM_H
