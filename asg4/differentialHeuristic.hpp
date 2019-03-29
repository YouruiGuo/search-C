//
//  differentialHeuristic.hpp
//  astar
//
//  Created by Yourui Guo on 2019-03-24.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#ifndef differentialHeuristic_h
#define differentialHeuristic_h

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
//#include <cstdlib>
#include <cmath>
#include <iterator>
#include <queue>
#include <map>
#include <cmath>
#include <sys/stat.h>
#include <unistd.h>
#include <random>
#include "voxel_env.hpp"
//#include "grid_env.hpp"

using namespace std;
typedef std::pair<float, unsigned long long> weight_vertex_pair;

class DifferentialHeuristic {
private:
    Voxel_env env = Voxel_env();
    unsigned long long numExpandNode, totalnum;
    std::vector<std::vector<float>> allHash;
    State lastState;
    int numPivots;
    std::vector<int> mapSize;

public:
    DifferentialHeuristic();
    DifferentialHeuristic(State s, State g);
    ~DifferentialHeuristic();
    Voxel_env getEnv();
    void buildHeuristic();
    void dijkstraSearch(std::vector<State> *s, int id);
    void randomPlacement();
    void furthestPlacement();
    void optimizedPlacement();
    double singleConsistentLookup(State st);
    double singleInconsistentLookup(State st);
    double multipleLookup(State st);
    unsigned long long lexicographicalRanking(State st);
    double HCost(State st);
};

DifferentialHeuristic::DifferentialHeuristic(State s, State g) {
    env = Voxel_env(s, g);
    numExpandNode = 0;
    numPivots = 10;
    totalnum = 1;
    mapSize = env.getMapsize();
    for (unsigned int i=0; i<mapSize.size(); i++) {
        totalnum *= (mapSize[i]+1);
    }
    allHash.resize(totalnum);
<<<<<<< HEAD
    for (unsigned int i = 0; i < totalnum; i++) {
        allHash[i].resize(30, 0);
=======
    for (int i = 0; i < totalnum; i++) {
        allHash[i].resize(30, env.inf);
>>>>>>> 3c27f3b8ca971fdcdf173029b7f564139be1263d
    }
    buildHeuristic();
}

DifferentialHeuristic::DifferentialHeuristic(){}
DifferentialHeuristic::~DifferentialHeuristic(){}

Voxel_env DifferentialHeuristic::getEnv() {
    return env;
}

void DifferentialHeuristic::buildHeuristic() {
    randomPlacement();
    //furthestPlacement();
    //optimizedPlacement();
}

void DifferentialHeuristic::dijkstraSearch(std::vector<State> *s, int id) {
    
    std::priority_queue<weight_vertex_pair,
                        std::vector<weight_vertex_pair>,
                        greater<weight_vertex_pair>> Q;
    
    std::vector<Action> actions;
    Action action = Action({-1});
    unsigned long long temphash, next_hash;
    float cost = 0;
    State temp;

    // push the start state to the priority queue
    for (unsigned int i = 0; i < s->size(); i++) {
        temphash = env.getStateHash((*s)[i]);
        Q.push(std::make_pair(0, temphash));
        allHash[temphash][id] = 0;
    }

    while (!Q.empty()) {
        numExpandNode++;
        temphash = Q.top().second;
<<<<<<< HEAD
        temp = env.allStates[env.hashtable[temphash]].getState();

=======
        //temp = env.allStates[env.hashtable[temphash]].getState();
        temp = env.unranking(temphash);
>>>>>>> 3c27f3b8ca971fdcdf173029b7f564139be1263d
        /*if (Q.top().first > maxgcost) {
            lastState = temp;
        }*/
        Q.pop();

        if (Q.empty()) {
            lastState = temp;
        }
        if (numExpandNode%10000 == 0) {
            std::cout << numExpandNode << endl;
        }

        env.getActions(temp, &actions);
        for (std::vector<Action>::size_type i = 0; i < actions.size(); ++i) {
            action = actions[i];
            // get the cost of the action
            cost = 0;
            for (auto& n : action.getAction()) {
                cost += abs(n);
            }
            cost = sqrt(cost);

            env.applyAction(action, &temp);
            next_hash = env.getStateHash(temp);

            /*tempgcost = -1;
            try {
                tempgcost = allHash[next_hash][id];
            } catch (...) {
                continue;
            }*/
            if (allHash[next_hash][id] > allHash[temphash][id] + cost) {
                allHash[next_hash][id] = allHash[temphash][id] + cost;
                Q.push(std::make_pair(allHash[next_hash][id], next_hash));
            }
            env.undoAction(action, &temp);
        }
    }
}

void DifferentialHeuristic::randomPlacement() {
    std::vector<int> tempstate;
    std::vector<State> t;
    unsigned long long hashtemp;
    bool isfilled = false;
    for (int i = 0; i < numPivots; i++) {
        cout << "build "<< i <<" heuristic " << endl;
        t.clear();
        tempstate.clear();
        srand( static_cast<unsigned int>(time(NULL)));
        do {
            isfilled = false;
            tempstate.push_back(rand()%mapSize[0]);
            tempstate.push_back(rand()%mapSize[1]);
            tempstate.push_back(rand()%mapSize[2]);
            hashtemp = env.getStateHash(State(tempstate));
            // make sure the random selected point is not an obstacle
            std::map<unsigned long long, int>::iterator f =
                               env.hashtable.find(hashtemp);
            if (f != env.hashtable.end()) {
                if (env.allStates[hashtemp].gcost == env.inf) {
                    isfilled = true;
                }
            }
        } while (isfilled);
        t.push_back(State(tempstate));
        dijkstraSearch(&t, i);
    }
}


void DifferentialHeuristic::furthestPlacement() {
    std::vector<int> tempstate;
    std::vector<State> t;
    unsigned long long hashtemp;
    bool isfilled = false;
    srand( static_cast<unsigned int>(time(NULL)));
    do {
        tempstate.push_back(rand()%mapSize[0]);
        tempstate.push_back(rand()%mapSize[1]);
        tempstate.push_back(rand()%mapSize[2]);
        hashtemp = env.getStateHash(State(tempstate));
        if (env.allStates[hashtemp].gcost == env.inf) {
            isfilled = true;
        }
    } while (!isfilled);
    t.push_back(State(tempstate));
    dijkstraSearch(&t, 0);
    tempstate = lastState.getState();
    for (int i = 0; i < 10; i++) {
        dijkstraSearch(&t, i);
        t.push_back(lastState);
    }
}

void DifferentialHeuristic::optimizedPlacement() {
    std::vector<int> tempstate;
    std::vector<State> t, samples;
    unsigned long long hashtemp;
    //unsigned long long t1, t2;
    bool isfilled = false;
    std::vector<double> heugains;
    heugains.resize(30, 0);
    srand( static_cast<unsigned int>(time(NULL)));
    do {
        tempstate.push_back(rand()%mapSize[0]);
        tempstate.push_back(rand()%mapSize[1]);
        tempstate.push_back(rand()%mapSize[2]);
        hashtemp = env.getStateHash(State(tempstate));
        if (env.allStates[hashtemp].gcost == env.inf) {
            isfilled = true;
        }
    } while (!isfilled);
    t.push_back(State(tempstate));
    dijkstraSearch(&t, 0);
    tempstate = lastState.getState();
    // generate 30 pivots and corresponding heuristics
    for (int i = 0; i < 30; i++) {
        dijkstraSearch(&t, i);
        t.push_back(lastState);
    }

    // generate samples
    for (int i = 0; i < 100; i++) {
        tempstate.clear();
        do {
            tempstate.push_back(rand()%mapSize[0]);
            tempstate.push_back(rand()%mapSize[1]);
            tempstate.push_back(rand()%mapSize[2]);
            hashtemp = env.getStateHash(State(tempstate));
            if (env.allStates[hashtemp].gcost == env.inf) {
                isfilled = true;
            }
        } while (!isfilled);
        samples.push_back(State(tempstate));
    }

    // measure the best 10 heuristics by comparing heuristic gains.
    for (int i = 0; i < 50; i++) {
        //t1 = env.getStateHash(samples[i*2]);
        //t2 = env.getStateHash(samples[i*2+1]);
        // TODO
    }

}

double DifferentialHeuristic::singleConsistentLookup(State st) {
    double value1 = 0, value2 = 0;
    int id = 0;
    unsigned long long h1 = env.getStateHash(st);
    unsigned long long h2 = env.getStateHash(env.getGoal());
    value1 = allHash[h1][id];
    value2 = allHash[h2][id];
    return abs(value1 - value2);
}

double DifferentialHeuristic::singleInconsistentLookup(State st) {
    double value1 = 0, value2 = 0;
    srand( static_cast<unsigned int>(time(NULL)));
    int id = rand()%numPivots;
    unsigned long long h1 = env.getStateHash(st);
    unsigned long long h2 = env.getStateHash(env.getGoal());
    value1 = allHash[h1][id];
    value2 = allHash[h2][id];
    return abs(value1 - value2);
}

double DifferentialHeuristic::multipleLookup(State st) {
    double value1 = 0, value2 = 0;
    unsigned long long h1 = env.getStateHash(st);
    unsigned long long h2 = env.getStateHash(env.getGoal());
    for (int i = 0; i < numPivots; i++) {
        if (allHash[h1][i] > value1) {
            value1 = allHash[h1][i];
        }
        if (allHash[h2][i] > value2) {
            value2 = allHash[h2][i];
        }
    }
    return abs(value1 - value2);
}

double DifferentialHeuristic::HCost(State st) {
    return singleInconsistentLookup(st);
    //return singleConsistentLookup(st);
    //return multipleLookup(st);
}


#endif /* differentialHeuristic_h */
