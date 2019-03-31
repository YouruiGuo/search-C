//
//  differentialHeuristic.hpp
//  astar
//
//  Created by Yourui Guo on 2019-03-24.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#ifndef differentialHeuristic_h
#define differentialHeuristic_h

#include <stdlib.h>
#include <stdio.h>
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
    std::vector<int> mapSize,tempstate1, tempstate2;

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
    float singleConsistentLookup(State st);
    float singleInconsistentLookup(State st);
    float multipleLookup(State st);
    unsigned long long lexicographicalRanking(State st);
    float HCost(State st);
    void writeToFile();
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

    for (unsigned int i = 0; i < totalnum; i++) {
        allHash[i].resize(30, env.inf);
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
        temp = env.unranking(temphash);
        Q.pop();
        if (Q.empty()) {
            lastState = temp;
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

            if (allHash[next_hash][id] > allHash[temphash][id] + cost) {
                allHash[next_hash][id] = allHash[temphash][id] + cost;
                Q.push(std::make_pair(allHash[next_hash][id], next_hash));
            }
            env.undoAction(action, &temp);
        }
    }
}

void DifferentialHeuristic::writeToFile() {
    std::string name = "./heuristic_furthest.txt";
    ofstream f(name);
    if (f.is_open()) {
        for (unsigned int i = 0; i < totalnum; ++i) {
            for (int j = 0; j < 10; ++j) {
                f << allHash[i][j] << ' ';
            }
            f << endl;
        }
    }
}

void DifferentialHeuristic::randomPlacement() {
    std::string name = "./heuristic_random.txt";
    struct stat buffer;
    if (stat(name.c_str(), &buffer) == 0) {
        std::string line;
        int count = 0;
        ifstream f(name);
        while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>()};
            for (int i = 0; i < 10; ++i) {
                allHash[count][i] = std::stof(tokens[i]);
            }
            count++;
        }
        //cout << "built" << endl;
    }
    else {
        std::vector<int> tempstate;
        std::vector<State> t;
        unsigned long long hashtemp;
        bool isfilled = false;
        for (int i = 0; i < numPivots; i++) {
            cout << "build "<< i <<" heuristic " << endl;
            t.clear();
            srand( static_cast<unsigned int>(time(NULL)));
            do {
                tempstate.clear();
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
            cout << "pivot generated" << endl;
            dijkstraSearch(&t, i);
        }
        writeToFile();
    }
}


void DifferentialHeuristic::furthestPlacement() {
    std::string name = "./heuristic_furthest.txt";
    struct stat buffer;
    if (stat(name.c_str(), &buffer) == 0) {
        std::string line;
        int count = 0;
        ifstream f(name);
        while (std::getline(f, line)) {
            std::istringstream iss(line);
            std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
                    std::istream_iterator<std::string>()};
            for (int i = 0; i < 10; ++i) {
                allHash[count][i] = std::stof(tokens[i]);
            }
            count++;
        }
        //cout << "built" << endl;
    }
    else {
        std::vector<int> tempstate;
        std::vector<State> t;
        t.clear();
        unsigned long long hashtemp;
        bool isfilled = false;
        srand( static_cast<unsigned int>(time(NULL)));
        do {
            tempstate.clear();
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
        dijkstraSearch(&t, 0);
        t.clear();
        cout << "random done" << endl;
        //tempstate = lastState.getState();
        for (int i = 0; i < 10; i++) {
            t.push_back(lastState);
            dijkstraSearch(&t, i);
            cout << "heuristic" << i << "built";
        }
        writeToFile();
    }
}

void DifferentialHeuristic::optimizedPlacement() {
    std::vector<int> tempstate;
    std::vector<State> t, samples;
    unsigned long long hashtemp;
    //unsigned long long t1, t2;
    bool isfilled = false;
    std::vector<float> heugains;
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

float DifferentialHeuristic::singleConsistentLookup(State st) {
    float value1 = 0, value2 = 0;
    int id = 0;
    unsigned long long h1 = env.getStateHash(st);
    unsigned long long h2 = env.getStateHash(env.getGoal());
    value1 = allHash[h1][id];
    value2 = allHash[h2][id];
    return fabs(value1 - value2);
}

float DifferentialHeuristic::singleInconsistentLookup(State st) {
    float value1 = 0, value2 = 0;
    srand( static_cast<unsigned int>(time(NULL)));
    int id = rand()%numPivots;
    unsigned long long h1 = env.getStateHash(st);
    unsigned long long h2 = env.getStateHash(env.getGoal());
    value1 = allHash[h1][id];
    value2 = allHash[h2][id];
    return fabs(value1 - value2);
}

float DifferentialHeuristic::multipleLookup(State st) {
    float value1 = 0, maxv = 0;
    unsigned long long h1 = env.getStateHash(st);
    unsigned long long h2 = env.getStateHash(env.getGoal());
    for (int i = 0; i < numPivots; i++) {
        value1 = fabs(allHash[h1][i] - allHash[h2][i]);
        if (maxv < value1) {
            maxv = value1;
        }
    }
    return maxv;
}

float DifferentialHeuristic::HCost(State st) {
    //return 0;
    return singleInconsistentLookup(st);
    //return singleConsistentLookup(st);
    //return multipleLookup(st);
    float x,y,z,dmin,dmax,dmid;
    tempstate1 = st.getState();
    tempstate2 = env.getGoal().getState();
    x = abs(tempstate1[0] - tempstate2[0]);
    y = abs(tempstate1[1] - tempstate2[1]);
    z = abs(tempstate1[2] - tempstate2[2]);
    dmin = fmin(x,y);
    dmin = fmin(dmin, z);
    dmax = fmax(x,y);
    dmax = fmax(dmax, z);

    dmid = x+y+z-dmin-dmax;
    return ((sqrt(3)-sqrt(2))*dmin + (sqrt(2)-1)*dmid + dmax);
}


#endif /* differentialHeuristic_h */
