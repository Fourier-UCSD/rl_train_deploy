#ifndef XFsmState_H
#define XFsmState_H

#include <stdio.h>
#include <string>
#include <map>

//#include  "ArmController.h"
// #include "XApp.h"

//typedef void (*TRANS_FUNC)(void* oldData, string event, void* newData);

using namespace std;
/**
 *
 */
class XFsmState {
public:
  //XFsmState(void* _robotData);
  XFsmState(void* app);

  virtual void init() = 0;// {}
  // Behavior to be carried out when entering a state
  virtual void onEnter() = 0;// {}

  // Run the normal behavior for the state
  virtual void run() = 0; //{}

  // Behavior to be carried out when exiting a state
  virtual void onExit() = 0; // {}


  string stateName;      // state name string

  virtual int transitionByEvent(string event) { return 0;}
  virtual int transitionByDuration() { return 0;}

  int addEventTrans(string event, string nextState);
  int addDurationTrans(int loops, string nextState);
  string transition(string event);
  int endDuration();
  void* app;

private:
  map<string , string> eventMap;
  int duration;	
  int elapse;	
  string durationState;
};

#endif  // XFsmState_H
