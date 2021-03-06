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
#include "state.hpp"

using namespace std;
/*
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
*/
class Stp_env {
private:
    unsigned int size;
    State start_state;
    State goal_state;
    
    //std::vector<int> aacs, issuccess, statehash, empty;
    
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
    State applyActionBFS(Action a, State st);
    void setStart(State s);
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

void Stp_env::setStart(State s) {
    start_state = s;
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
    std::vector<int> issuccess = st.getState();
    for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
        // cout << issuccess[i] << ' ';
        if (issuccess[i] != (int)i) {
            return false;
        }
    }
    // cout << endl;
    return true;
}

bool Stp_env::isRepeat(Action prev, Action curr) {
    if (abs(prev.getAction() - curr.getAction()) == 2) {
        return true;
    }
    return false;
}

unsigned long long Stp_env::getStateHash(State st) {
    std::vector<int> statehash = st.getState();
    unsigned long long value = 0;
    for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
        value = value << 4;
        if (statehash[i] != -1) {
            value += statehash[i];
        }
    }
    return value;
}

int Stp_env::getEmpty(State st) {
    std::vector<int> empty = st.getState();
    for (std::vector<int>::size_type i = 0; i < size*size; ++i) {
        if (empty[i] == 0) {
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
    std::vector<int> aacs = st.getState();
    int empty = getEmpty(st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
    if(a.getAction() == 0) {
        aacs[row*size+col] = aacs[(row-1)*size+col];
        aacs[(row-1)*size+col] = 0;
    }
    else if (a.getAction() == 1) {
        aacs[row*size+col] = aacs[row*size+col+1];
        aacs[row*size+col+1] = 0;
    }
    else if (a.getAction() == 2) {
        aacs[row*size+col] = aacs[(row+1)*size+col];
        aacs[(row+1)*size+col] = 0;
    }
    else if (a.getAction() == 3) {
        aacs[row*size+col] = aacs[row*size+col-1];
        aacs[row*size+col-1] = 0;
    }
    State new_st = State(aacs);
    unsigned long long hv = getStateHash(new_st);
    allStates.push_back(StateInfo(new_st));
    hashtable[hv] = (int)allStates.size()-1;
    return hashtable[hv];
}

State Stp_env::applyActionBFS(Action a, State st) {
    std::vector<int> aacs = st.getState();
    int empty = getEmpty(st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
    if(a.getAction() == 0) {
        aacs[row*size+col] = aacs[(row-1)*size+col];
        aacs[(row-1)*size+col] = 0;
    }
    else if (a.getAction() == 1) {
        aacs[row*size+col] = aacs[row*size+col+1];
        aacs[row*size+col+1] = 0;
    }
    else if (a.getAction() == 2) {
        aacs[row*size+col] = aacs[(row+1)*size+col];
        aacs[(row+1)*size+col] = 0;
    }
    else if (a.getAction() == 3) {
        aacs[row*size+col] = aacs[row*size+col-1];
        aacs[row*size+col-1] = 0;
    }
    return State(aacs);
}

void Stp_env::applyAction(Action a, State *st) {////
    std::vector<int> aacs = st->getState();
    int empty = getEmpty(*st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
    if(a.getAction() == 0) {
        aacs[row*size+col] = aacs[(row-1)*size+col];
        aacs[(row-1)*size+col] = 0;
    }
    else if (a.getAction() == 1) {
        aacs[row*size+col] = aacs[row*size+col+1];
        aacs[row*size+col+1] = 0;
    }
    else if (a.getAction() == 2) {
        aacs[row*size+col] = aacs[(row+1)*size+col];
        aacs[(row+1)*size+col] = 0;
    }
    else if (a.getAction() == 3) {
        aacs[row*size+col] = aacs[row*size+col-1];
        aacs[row*size+col-1] = 0;
    }
    st->setState(aacs);
    
}

void Stp_env::undoAction(Action a, State *st) {
    std::vector<int> aacs = st->getState();
    int empty = getEmpty(*st);
    int row = (int)(empty/size);
    int col = (int)(empty%size);
    if(a.getAction() == 0) { // down
        aacs[row*size+col] = aacs[(row+1)*size+col];
        aacs[(row+1)*size+col] = 0;
    }
    else if (a.getAction() == 1) { // left
        aacs[row*size+col] = aacs[row*size+col-1];
        aacs[row*size+col-1] = 0;
    }
    else if (a.getAction() == 2) { // up
        aacs[row*size+col] = aacs[(row-1)*size+col];
        aacs[(row-1)*size+col] = 0;
    }
    else if (a.getAction() == 3) { // right
        aacs[row*size+col] = aacs[row*size+col+1];
        aacs[row*size+col+1] = 0;
    }
    st->setState(aacs);
}

int Stp_env::cost(State prev, State curr) {
    return 1;
}


#endif /* stp_env_h */
