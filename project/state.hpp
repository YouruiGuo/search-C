//
//  state.hpp
//  astar
//
//  Created by Yourui Guo on 2019-03-24.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#ifndef state_h
#define state_h

class State {
    std::vector<int> state;
    
public:
    State();
    State(std::vector<int> v);
    ~State();
    std::vector<int> getState();
    void setState(std::vector<int> s);
};

State::State(std::vector<int> v){
    state = v;
}
State::State(){}
State::~State(){}


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
    float gcost;
    float hcost;
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
    gcost = std::numeric_limits<double>::infinity();
    open_id = -10;
    parent = -1;
}
StateInfo::StateInfo(){}
StateInfo::~StateInfo(){}
State StateInfo::getState(){
    return state;
}

#endif /* state_h */
