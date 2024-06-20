#include "../include/XFsmMgr.h"
#include "../include/XFsmState.h"
// #include "XFsmMgr.h"
// #include "XFsmState.h"
XFsmMgr::XFsmMgr()
{
  // Initialize the FSM with the Passive FSM State
//  initialize();
}


XFsmState* XFsmMgr::getStateByName(string name)
{
	
    map<string, XFsmState*>::iterator it = XFsmStatesMap.find(name);
    if(it == XFsmStatesMap.end()) {
		return NULL;
    } else {
		return it->second;
    }
}

void XFsmMgr::addState(XFsmState* state)
{
	addState(state, 0);
}

void XFsmMgr::addState(XFsmState* state, int firstflag)
{
	map<string, XFsmState*>::iterator it = XFsmStatesMap.find(state->stateName);
    if(it == XFsmStatesMap.end()) {
        XFsmStatesMap.insert(std::pair<string, XFsmState*>(state->stateName, state));
    } else {
        it->second = state;
    }

	if(firstflag)
		firstStateName = state->stateName;
	return;
}

void XFsmMgr::initialize() {
	std::cout << "initialize" << std::endl;
	
  // Initialize a new FSM State with the control data
  currentState = getStateByName(firstStateName);

//	printf("%s %d %s\n", __func__, __LINE__, currentState->stateName.c_str());
  currentState->onEnter();


  nextState = currentState;

}

void XFsmMgr::runFSM(string& event) {
	string state;
/*	
	static int count =0;

	if(count == 999) {
		printf("%s %d %s\n", __func__, __LINE__, currentState->stateName.c_str());
		count = 0;
	} else {
		count++;
	}
*/
	state = currentState->transition(event);
	event = "";
	
	if(state.size() == 0) {

		currentState->run();
		return;
	}

	if (state == currentState->stateName) {
		currentState->run();
		return;

    }
	
	nextState = getStateByName(state);
	printf("current %s \n", currentState->stateName.c_str());
	printf("nextstate %s\n", nextState->stateName.c_str());

	currentState->onExit();

	currentState = nextState;

	currentState->onEnter();
	currentState->run();

	return;
}
// return currentstate name
void XFsmMgr::currentstatename(string& state)
{
    state = currentState->stateName;
}

