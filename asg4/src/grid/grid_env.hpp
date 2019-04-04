//
//  grid_env.hpp
//  astar
//
//  Created by Yourui Guo on 2019-03-25.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#ifndef grid_env_h
#define grid_env_h

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iterator>
#include <map>
#include <cmath>
#include "state.hpp"


class Action
{
private:
    std::vector<int> action;
public:
    Action(std::vector<int> a);
    ~Action();
    std::vector<int> getAction();
    void setAction(std::vector<int> a);
};

Action::Action(std::vector<int> a) {
    action = a;
}

Action::~Action(){}

std::vector<int> Action::getAction() {
    return action;
}

void Action::setAction(std::vector<int> a) {
    action = a;
}

class Grid_env {
    State start_state;
    State goal_state;
    std::vector<int> mapsize; // mapsize[0]:height mapsize[1]: width
    std::vector<Action> allActions, singleAction;
    
    float gcost;
    
    std::vector<int> isuccess_t, isuccess_g, inmap, hashtemp, costp, costc;
    std::vector<int> isrepeat_p, isrepeat_c, lx, ly, lz, act;
    
public:
    double inf;
    std::map<unsigned long long, int> hashtable;
    std::vector<StateInfo> allStates;
    Grid_env(State s, State g);
    Grid_env();
    ~Grid_env();
    void setStart(State s);
    void setGoal(State g);
    State getStart();
    State getGoal();
    bool isSuccess(State st);
    bool isRepeat(Action prev, Action curr);
    bool isInMap(State st);
    unsigned long long getStateHash(State st);
    void getActions(State st, std::vector<Action> *actions);
    int applyActionCopy(Action a, State s); //return index of stateinfo list
    void applyAction(Action a, State *s); //return cost
    void undoAction(Action a, State *s);
    double cost(State prev, State curr);
    void loadMap();
    std::vector<int> getMapsize();
    int getHashIndex(unsigned long long i);
    State unranking(unsigned long long h);
};

Grid_env::Grid_env(){}
Grid_env::~Grid_env(){}

Grid_env::Grid_env(State s, State g) {
    start_state = s;
    goal_state = g;
    //mapsize = si;
    inf = std::numeric_limits<double>::infinity();
    mapsize.resize(2, 0);
    loadMap();
    
    allStates.push_back(s);
    unsigned long long svalue = getStateHash(s);
    hashtable[svalue] = (int)allStates.size()-1;
    
    allStates.push_back(g);
    unsigned long long gvalue = getStateHash(g);
    hashtable[gvalue] = (int)allStates.size()-1;
    
    std::vector<int> v = {-1, 0, 1};
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            if (v[i]==0 && v[j]==0) {
                continue;
            }
            allActions.push_back(Action({v[i],v[j]}));
        }
    }
    
}

void Grid_env::setStart(State s) {
    start_state = s;
}

void Grid_env::setGoal(State g) {
    goal_state = g;
}

State Grid_env::getStart() {
    return start_state;
}

State Grid_env::getGoal() {
    return goal_state;
}

bool Grid_env::isSuccess(State st) {
    isuccess_t = st.getState();
    isuccess_g = goal_state.getState();
    for (unsigned int i=0; i<2; i++) {
        if (isuccess_t[i] != isuccess_g[i]) {
            return false;
        }
    }
    return true;
}

bool Grid_env::isRepeat(Action prev, Action curr) {
    int count = 0;
    isrepeat_p = prev.getAction();
    isrepeat_c = curr.getAction();
    if (isrepeat_p[0] == -isrepeat_c[0]) {
        count++;
    }
    if (isrepeat_p[1] == -isrepeat_c[1]) {
        count++;
    }
    if (count == 2) {
        return true;
    }
    return false;
}

bool Grid_env::isInMap(State st) {
    inmap = st.getState();
    if ((inmap[0] < 0) || (inmap[0] >= mapsize[0])) {
        return false;
    }
    if ((inmap[1] < 0) || (inmap[1] >= mapsize[1])) {
        return false;
    }
    return true;
}

unsigned long long Grid_env::getStateHash(State st) {
    hashtemp = st.getState();
    unsigned long long value = 0;
    value = hashtemp[0]*mapsize[1]+hashtemp[1];
    return value;
}

State Grid_env::unranking(unsigned long long h) {
    hashtemp[1] = h % mapsize[1];
    hashtemp[0] = int(h / mapsize[1]);
    return State(hashtemp);
}

void Grid_env::getActions(State st, std::vector<Action> *actions) {
    actions->clear();
    bool filled;
    unsigned long long hv;
    for (unsigned int i=0; i<allActions.size(); i++) {
        singleAction.clear();
        lx.clear();
        lx.push_back(0);
        ly.clear();
        ly.push_back(0);
        act = allActions[i].getAction();
        if (act[0] != 0) {
            lx.push_back(act[0]);
        }
        if (act[1] != 0) {
            ly.push_back(act[1]);
        }
        filled = false;
        for (const int x : lx) {
            for (const int y : ly) {
                if (x==0 && y==0) {
                    continue;
                }
                applyAction(Action({x,y}), &st);
                hv = getStateHash(st);
                gcost = -1;
                std::map<unsigned long long, int>::iterator f =
                hashtable.find(hv);
                if (f != hashtable.end()) {
                    gcost = allStates[hashtable[hv]].gcost;
                }
                if (gcost == inf) {
                    filled = true;
                }
                if (!isInMap(st)) {
                    filled = true;
                }
                undoAction(Action({x,y}), &st);
            }
        }
        if (!filled) {
            actions->push_back(allActions[i]);
        }
    }
}

int Grid_env::applyActionCopy(Action a, State s) {
    std::vector<int> tempst = s.getState();
    tempst[0] += a.getAction()[0];
    tempst[1] += a.getAction()[1];
    State newst = State(tempst);
    unsigned long long hv = getStateHash(newst);
    std::map<unsigned long long, int>::iterator f =
    hashtable.find(hv);
    if (f != hashtable.end()) {
        return hashtable[hv];
    }
    allStates.push_back(StateInfo(newst));
    hashtable[hv] = (int)allStates.size()-1;
    return hashtable[hv];
}

void Grid_env::applyAction(Action a, State *s) {
    std::vector<int> tempst = s->getState();
    tempst[0] += a.getAction()[0];
    tempst[1] += a.getAction()[1];
    s->setState(tempst);
}

void Grid_env::undoAction(Action a, State *s) {
    std::vector<int> tempst = s->getState();
    tempst[0] -= a.getAction()[0];
    tempst[1] -= a.getAction()[1];
    s->setState(tempst);
}

void Grid_env::loadMap() {
    std::vector<int> s;
    s.resize(2, 0);
    State st;
    unsigned long long h;
    std::string filename = "/Users/margaret/Documents/cmput652/search-C1/asg4/data/brc101d.map";
    std::ifstream src (filename);

    int line = 0;
    std::string input;
    while (line < 4 && std::getline(src, input)) {
        if (line == 1 || line == 2){
            std::stringstream iss(input);
            std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
                std::istream_iterator<std::string>()};
            mapsize[2-line] = std::stoi(tokens[1]);
        }
        line++;
    }
    line = 0;
    while (src && line < mapsize[0]) {
        std::getline (src, input);
        for (size_t i = 0; i < mapsize[1]; i++) {
            if (input[i] != '.') {
                s.clear();
                s.push_back((int)i);
                s.push_back(line);
                st = State(s);
                h = getStateHash(st);
                allStates.push_back(StateInfo(st));
                hashtable[h] = (int)allStates.size() - 1;
                allStates[hashtable[h]].gcost = inf;
            }
        }
        line++;
    }
}

double Grid_env::cost(State prev, State curr) {
    double value = 0;
    costc = curr.getState();
    costp = prev.getState();
    value += abs(costc[0] - costp[0]);
    value += abs(costc[1] - costp[1]);
    return sqrt(value);
}

int Grid_env::getHashIndex(unsigned long long i) {
    return hashtable[i];
}


std::vector<int> Grid_env::getMapsize() {
    return mapsize;
}


#endif /* grid_env_h */
