/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "../include/XFsmState.h"
// #include "XFsmState.h"

//XFsmState::XFsmState(void* robotData) : _robotData(robotData)
XFsmState::XFsmState(void* app) : app(app)
{
	duration = -1;
	elapse = -1;
}

int XFsmState::addEventTrans(string event, string nextState)
{
	map<string, string>::iterator it = eventMap.find(event);

	if(it != eventMap.end()) {
		eventMap[event] = nextState;
	} else {
		eventMap.insert(pair<string, string>(event, nextState));
	}

	return 0;	
}

int XFsmState::addDurationTrans(int loops, string nextState)
{
	duration = loops;
	elapse = loops;
	durationState = nextState;
	return 0;
}


int XFsmState::endDuration()
{
	if(elapse > 0) {
		elapse = 0;
		return 0;
	} else {
		return -1;
	}
}

string XFsmState::transition(string event)
{
	if(elapse > 0) {
		elapse--;
		return string();
	} else if(elapse == 0) {
		elapse = duration;
		return durationState;
	}

	//printf("%s %d %s\n", __func__, __LINE__, event.c_str());
	map<string, string>::iterator it = eventMap.find(event);

	if(it != eventMap.end()) {
		return it->second;	
	}

	return string();
	

}
