//
//  stp_env.hpp
//  search-C
//
//  Created by Yourui Guo on 2019-03-08.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#ifndef stp_env_h
#define stp_env_h

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
public:
    int gcost;
    int hcost;
    int open_id;
    int parent;
    StateInfo();
    StateInfo(State s);
    ~StateInfo();
    State getState();
};

StateInfo::StateInfo(State s){
    state = s;
    hcost = 0;
    gcost = 0;
    open_id = -10;
    parent = -1;
}
StateInfo::StateInfo(){}
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
    State start_state;
    State goal_state;
    
    
    
public:
    std::map<unsigned long long, int> hashtable;
    std::vector<StateInfo> allStates;
    Stp_env();
    Stp_env(State s, State g);
    ~Stp_env();
    bool isSuccess(State st);
    int getEmpty(State st);
    bool isRepeat(Action prev, Action curr);
    unsigned long long getStateHash(State st);
    void getActions(State st, std::vector<Action> *actions);
    int applyActionCopy(Action a, State s);
    void applyAction(Action a, State *s);
    void undoAction(Action a, State *s);
    int cost(State prev, State curr);
    unsigned int getSize();
    int getHashIndex(unsigned long long i);
    State getStart();
    State getGoal();
};

Stp_env::Stp_env(){}
Stp_env::~Stp_env(){}
Stp_env::Stp_env(State s, State g) {
    size = 4;
    start_state = s;
    goal_state = g;
    //StateInfo start = StateInfo(s);
    //StateInfo goal = StateInfo(g);
    
    allStates.push_back(s);
    unsigned long long svalue = getStateHash(s);
    hashtable[svalue] = 0;
    
    allStates.push_back(g);
    unsigned long long gvalue = getStateHash(g);
    hashtable[gvalue] = 1;
}

unsigned int Stp_env::getSize() {
    return size;
}

State Stp_env::getStart() {
    return start_state;
}

State Stp_env::getGoal() {
    return goal_state;
}

int Stp_env::getHashIndex(unsigned long long i) {
    return hashtable[i];
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
    if (abs(prev.getAction() - curr.getAction()) == 2) {
        return true;
    }
    return false;
}

unsigned long long Stp_env::getStateHash(State st) {
    std::vector<int> s = st.getState();
    unsigned long long value = 0;
    for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
        value = value << 4;
        if (s[i] != -1) {
            value += s[i];
        }
    }
    return value;
}

int Stp_env::getEmpty(State st) {
    std::vector<int> s = st.getState();
    for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
        if (s[i] == 0) {
            return (int)i;
        }
    }
    return -1;
}

void Stp_env::getActions(State st, std::vector<Action> *actions) {
    //const std::vector<int> &s = st.getState();
    //    std::vector<Action> actions;
    actions->clear();
    int empty = getEmpty(st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
    
    if (row == 0) actions->push_back(Action(2));
    else if (row == (int)size-1) actions->push_back(Action(0));
    else{
        actions->push_back(Action(0));
        actions->push_back(Action(2));
    }
    
    if (col == 0) actions->push_back(Action(1));
    else if (col == (int)size-1) actions->push_back(Action(3));
    else {
        actions->push_back(Action(1));
        actions->push_back(Action(3));
    }
}

int Stp_env::applyActionCopy(Action a, State st) {
    std::vector<int> s = st.getState();
    int empty = getEmpty(st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
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
    unsigned long long hv = getStateHash(new_st);
    allStates.push_back(StateInfo(new_st));
    hashtable[hv] = (int)allStates.size()-1;
    return hashtable[hv];
}

void Stp_env::applyAction(Action a, State *st) {////
    std::vector<int> s = st->getState();
    int empty = getEmpty(*st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
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
    st->setState(s);
    
}

void Stp_env::undoAction(Action a, State *st) {
    std::vector<int> s = st->getState();
    int empty = getEmpty(*st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
    if(a.getAction() == 0) { // down
        s[row*size+col] = s[(row+1)*size+col];
        s[(row+1)*size+col] = 0;
    }
    else if (a.getAction() == 1) { // left
        s[row*size+col] = s[row*size+col-1];
        s[row*size+col-1] = 0;
    }
    else if (a.getAction() == 2) { // up
        s[row*size+col] = s[(row-1)*size+col];
        s[(row-1)*size+col] = 0;
    }
    else if (a.getAction() == 3) { // right
        s[row*size+col] = s[row*size+col+1];
        s[row*size+col+1] = 0;
    }
    st->setState(s);
}

int Stp_env::cost(State prev, State curr) {
    return 1;
}


#endif /* stp_env_h */
