#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iterator>
#include <map>

using namespace std;

class State
{
private:
	std::vector<int> state;
public:
	State();
	State(std::vector<int> v);
	~State();
	std::vector<int> getState();
	void setState(std::vector<int> s);
};

State::State(){
	std::vector<int> v = {0};
	state = v;
}

State::~State() {}

State::State(std::vector<int> v){
	state = v;
}

std::vector<int> State::getState() {
	return state;
}

void State::setState(std::vector<int> s) {
	state = s;
}

class StateInfo
{
private:
	State state;
	int gcost;
	int hcost;
	int open_id;
	int parent;
public:
	StateInfo(State s);
	State getState();
	~StateInfo();
	
};

StateInfo::StateInfo(State s){
	state = s;
	hcost = 0;
	gcost = 0;
	open_id = -10;
	parent = -1;
}

StateInfo::~StateInfo(){}

State StateInfo::getState(){
	return state;
}


class Action
{
private:
	int action;
public:
	Action(int a);
	~Action();
	int getAction();
	void setAction(int a);
};

Action::Action(int a) {
	action = a;
}

Action::~Action(){}

int Action::getAction() {
	return action;
}

void Action::setAction(int a) {
	action = a;
}

class Stp_env {
private:
	unsigned int size;
	State start;
	State goal;
	std::map<unsigned long long, int> hashtable;
	std::vector<StateInfo> allStates;

public:
	Stp_env(State s, State g);
	~Stp_env();
	bool isSuccess(State st);
	int getEmpty(State st);
	bool isRepeat(Action prev, Action curr);
	unsigned long long getStateHash(State st);
	std::vector<Action> getActions(State st);
	State applyActionCopy(Action a, State s);
	void applyAction(Action a, State s);
	void undoAction(Action a, State s);
	int cost(State prev, State curr);
	unsigned int getSize();
};

Stp_env::Stp_env(State s, State g) {
	size = 4;
	StateInfo start = StateInfo(s);
	StateInfo goal = StateInfo(g);
	allStates.push_back(s);
	allStates.push_back(g);
	unsigned long long svalue = getStateHash(s);
	hashtable[svalue] = 0;
	unsigned long long gvalue = getStateHash(g);
	hashtable[gvalue] = 1;
}

unsigned int Stp_env::getSize() {
	return size;
}

bool Stp_env::isSuccess(State st) {
	std::vector<int> s = st.getState();
	for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
		if (s[i] != (int)i) {
			return false;
		}
	}
	return true; 
}

bool Stp_env::isRepeat(Action prev, Action curr) {
	if (abs(prev.getAction() - curr.getAction()) == 0) {
		return true;
	}
	return false;
}

unsigned long long Stp_env::getStateHash(State st) {
	std::vector<int> s = st.getState();
	unsigned long long value = 0;
	for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
		value = value << 4;
		value += s[i];
	}
	return value;
}

int Stp_env::getEmpty(State st) {
	std::vector<int> s = st.getState();
	for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
		if (s[i] == 0) {
			return i;
		}
	}
	return -1;
} 

std::vector<Action> Stp_env::getActions(State st) {
	std::vector<int> s = st.getState();
	std::vector<Action> actions;
	int empty = getEmpty(st);
	unsigned int row = empty/size;
	unsigned int col = empty%size;

	if (row == 0) actions.push_back(Action(2));
	else if (row == size-1) actions.push_back(Action(0));
	else{
		actions.push_back(Action(0));
		actions.push_back(Action(2));
	}

	if (col == 0) actions.push_back(Action(1));
	else if (col == size-1) actions.push_back(Action(3));
	else {
		actions.push_back(Action(1));
		actions.push_back(Action(3));	
	}

	return actions;

}

State Stp_env::applyActionCopy(Action a, State st) {
	std::vector<int> s = st.getState();
	int empty = getEmpty(st);
	int row = empty/size;
	int col = empty%size;
	if(a.getAction() == 0) {
		s[row*size+col] = s[(row-1)*size+col];
		s[(row-1)*size+col] = 0;
	}
	else if (a.getAction() == 1) {
		s[row*size+col] = s[row*size+col+1];
		s[row*size+col+1] = 0; 
	}
	else if (a.getAction() == 2) {
		s[row*size+col] = s[(row+1)*size+col];
		s[(row+1)*size+col] = 0;
	}
	else if (a.getAction() == 3) {
		s[row*size+col] = s[row*size+col-1];
		s[row*size+col-1] = 0;
	}
	State new_st = State(s);
	unsigned long long value = getStateHash(st);
	std::map<unsigned long long, int>::iterator
								f = hashtable.find(value);
	if (f != hashtable.end()) {
		int id = hashtable[value];
		return allStates[id].getState();// 
	}
	allStates.push_back(StateInfo(new_st));
	hashtable[value] = allStates.size()-1;
	return new_st;
}

void Stp_env::applyAction(Action a, State st) {////
	std::vector<int> s = st.getState();
	int empty = getEmpty(st);
	int row = empty/size;
	int col = empty%size;
	if(a.getAction() == 0) {
		s[row*size+col] = s[(row-1)*size+col];
		s[(row-1)*size+col] = 0;
	}
	else if (a.getAction() == 1) {
		s[row*size+col] = s[row*size+col+1];
		s[row*size+col+1] = 0; 
	}
	else if (a.getAction() == 2) {
		s[row*size+col] = s[(row+1)*size+col];
		s[(row+1)*size+col] = 0;
	}
	else if (a.getAction() == 3) {
		s[row*size+col] = s[row*size+col-1];
		s[row*size+col-1] = 0;
	}
	st.setState(s);

}

void Stp_env::undoAction(Action a, State st) {
	std::vector<int> s = st.getState();
	int empty = getEmpty(st);
	int row = empty/size;
	int col = empty%size;
	if(a.getAction() == 0) {
		s[row*size+col] = s[(row+1)*size+col];
		s[(row+1)*size+col] = 0;
	}
	else if (a.getAction() == 1) {
		s[row*size+col] = s[row*size+col-1];
		s[row*size+col-1] = 0; 
	}
	else if (a.getAction() == 2) {
		s[row*size+col] = s[(row-1)*size+col];
		s[(row-1)*size+col] = 0;
	}
	else if (a.getAction() == 3) {
		s[row*size+col] = s[row*size+col+1];
		s[row*size+col+1] = 0;
	}
	st.setState(s);
}

int Stp_env::cost(State prev, State curr) {
	return 1;
}

