//
//  voxel_env.hpp
//  astar
//
//  Created by Yourui Guo on 2019-03-24.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#ifndef voxel_env_h
#define voxel_env_h
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

class Voxel_env {
    State start_state;
    State goal_state;
    std::vector<int> mapsize;
    std::vector<Action> allActions, singleAction;

    std::vector<int> isuccess_t, isuccess_g, inmap, hashtemp, costp, costc;
    std::vector<int> isrepeat_p, isrepeat_c, lx, ly, lz, act;



public:
    double inf;
    std::map<unsigned long long, int> hashtable;
    std::vector<StateInfo> allStates;
    Voxel_env();
    Voxel_env(State s, State g);
    ~Voxel_env();
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
    float cost(State prev, State curr);
    void loadMap();
    int getHashIndex(unsigned long long i);
    std::vector<int> getMapsize();
    State unranking(unsigned long long h);
};

Voxel_env::Voxel_env(){}
Voxel_env::~Voxel_env(){}

Voxel_env::Voxel_env(State s, State g) {
    start_state = s;
    goal_state = g;
    inf = std::numeric_limits<double>::infinity();

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
            for (int k=0; k<3; k++) {
                if (v[i]==0 && v[j]==0 && v[k]==0) {
                    continue;
                }
                allActions.push_back(Action({v[i],v[j],v[k]}));
            }
        }
    }

}

void Voxel_env::setStart(State s) {
    start_state = s;
}

void Voxel_env::setGoal(State g) {
    goal_state = g;
}

State Voxel_env::getStart() {
    return start_state;
}

State Voxel_env::getGoal() {
    return goal_state;
}

bool Voxel_env::isSuccess(State st) {
    isuccess_t = st.getState();
    isuccess_g = goal_state.getState();
    for (unsigned int i=0; i<3; i++) {
        if (isuccess_t[i] != isuccess_g[i]) {
            return false;
        }
    }
    return true;
}

bool Voxel_env::isRepeat(Action prev, Action curr) {
    int count = 0;
    isrepeat_p = prev.getAction();
    isrepeat_c = curr.getAction();
    if (isrepeat_p[0] == -isrepeat_c[0]) {
        count++;
    }
    if (isrepeat_p[1] == -isrepeat_c[1]) {
        count++;
    }
    if (isrepeat_p[2] == -isrepeat_c[2]) {
        count++;
    }
    if (count == 3) {
        return true;
    }
    return false;
}

bool Voxel_env::isInMap(State st) {
    inmap = st.getState();
    if ((inmap[0] < 0) || (inmap[0] > mapsize[0])) {
        return false;
    }
    if ((inmap[1] < 0) || (inmap[1] > mapsize[1])) {
        return false;
    }
    if ((inmap[2] < 0) || (inmap[2] > mapsize[2])) {
        return false;
    }
    return true;
}

unsigned long long Voxel_env::getStateHash(State st) {
    hashtemp = st.getState();
    unsigned long long value = 0;
    value = (hashtemp[0]*mapsize[1]+hashtemp[1])*mapsize[2]+hashtemp[2];
    return value;
}

State Voxel_env::unranking(unsigned long long h) {
    hashtemp[2] = h % mapsize[2];
    h = h / mapsize[2];
    hashtemp[1] = h % mapsize[1];
    hashtemp[0] = int(h / mapsize[1]);
    return State(hashtemp);
}

void Voxel_env::getActions(State st, std::vector<Action> *actions) {
    actions->clear();
    bool filled;
    unsigned long long hv;
    float gcost;
    std::vector<int> lx, ly, lz;
    for (unsigned int i=0; i<allActions.size(); i++) {
        singleAction.clear();
        lx.clear();
        lx.push_back(0);
        ly.clear();
        ly.push_back(0);
        lz.clear();
        lz.push_back(0);
        act = allActions[i].getAction();
        if (act[0] != 0) {
            lx.push_back(act[0]);
        }
        if (act[1] != 0) {
            ly.push_back(act[1]);
        }
        if (act[2] != 0) {
            lz.push_back(act[2]);
        }
        filled = false;
        for (const int x : lx) {
            for (const int y : ly) {
                for (const int z : lz) {
                    if (x==0 && y==0 && z==0) {
                        continue;
                    }
                    applyAction(Action({x,y,z}), &st);
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
                    undoAction(Action({x,y,z}), &st);
                }
            }
        }
        if (!filled) {
            actions->push_back(allActions[i]);
        }
    }
}

int Voxel_env::applyActionCopy(Action a, State s) {
    std::vector<int> tempst = s.getState();
    tempst[0] += a.getAction()[0];
    tempst[1] += a.getAction()[1];
    tempst[2] += a.getAction()[2];
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

void Voxel_env::applyAction(Action a, State *s) {
    std::vector<int> tempst = s->getState();
    tempst[0] += a.getAction()[0];
    tempst[1] += a.getAction()[1];
    tempst[2] += a.getAction()[2];
    s->setState(tempst);
}

void Voxel_env::undoAction(Action a, State *s) {
    std::vector<int> tempst = s->getState();
    tempst[0] -= a.getAction()[0];
    tempst[1] -= a.getAction()[1];
    tempst[2] -= a.getAction()[2];
    s->setState(tempst);
}

void Voxel_env::loadMap() {
    std::vector<int> s;
    State st;
    unsigned long long h;
    std::string df = "/Users/margaret/Documents/cmput652/searchAlg/data/Simple.3dmap";
    std::ifstream infile(df);
    std::string line;
    std::getline(infile, line);
    std::istringstream iss(line);
    std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
        std::istream_iterator<std::string>()};
    mapsize.push_back(std::stoi(tokens[1]));
    mapsize.push_back(std::stoi(tokens[2]));
    mapsize.push_back(std::stoi(tokens[3]));
    while (std::getline(infile, line)) {
        s.clear();
        std::istringstream iss(line);
        std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
            std::istream_iterator<std::string>()};
        s.push_back(std::stoi(tokens[0]));
        s.push_back(std::stoi(tokens[1]));
        s.push_back(std::stoi(tokens[2]));
        st = State(s);
        h = getStateHash(st);
        allStates.push_back(StateInfo(st));
        hashtable[h] = (int)allStates.size() - 1;
        allStates[hashtable[h]].gcost = inf;
    }
}

float Voxel_env::cost(State prev, State curr) {
    float value = 0;
    costc = curr.getState();
    costp = prev.getState();
    value += abs(costc[0] - costp[0]);
    value += abs(costc[1] - costp[1]);
    value += abs(costc[2] - costp[2]);
    return sqrt(value);
}

int Voxel_env::getHashIndex(unsigned long long i) {
    return hashtable[i];
}


std::vector<int> Voxel_env::getMapsize() {
    return mapsize;
}

#endif /* voxel_env_h */
