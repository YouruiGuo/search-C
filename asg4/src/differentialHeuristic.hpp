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
    float voxelHeuristic(State st);
    float voxelHeuristic(State st1, State st2);
    void writeToFile(std::vector<int> v);
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
    //randomPlacement();
    //furthestPlacement();
    optimizedPlacement();
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
        temp = (*s)[i];
        cout << "in dijkstraSearch" << temp.getState()[0] << ' ' << temp.getState()[1] << ' ' << temp.getState()[2] << endl;
    }
    while (!Q.empty()) {
        numExpandNode++;
        temphash = Q.top().second;
        temp = env.unranking(temphash);
        if (temp.getState()[0] > env.getMapsize()[0] ||
            temp.getState()[1] > env.getMapsize()[1] || temp.getState()[2] > env.getMapsize()[2]){
            cout << temp.getState()[0] << ' ' << temp.getState()[1] << ' ' << temp.getState()[2] << endl;
        }
        Q.pop();
        if (Q.empty()) {
            lastState = temp;
            //cout << temp.getState()[0] << ' ' << temp.getState()[1] << ' ' << temp.getState()[2] << endl;
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

void DifferentialHeuristic::writeToFile(std::vector<int> v) {
    std::string name = "../heuristic_optimized.txt";
    ofstream f(name);
    int k = 0;
    if (f.is_open()) {
        for (unsigned int i = 0; i < totalnum; ++i) {
            k = 0;
            for (int j = 0; j < 30; ++j) {
                if (j == v[k]) {
                    f << allHash[i][j] << ' ';
                    k++;
                }
            }
            f << endl;
        }
    }
}

void DifferentialHeuristic::randomPlacement() {
    //cout << "random Placement" << endl;
    std::string name = "../heuristic_random.txt";
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
                    if (env.allStates[f->second].gcost == env.inf) {
                        isfilled = true;
                    }
                }
            } while (isfilled);
            t.push_back(State(tempstate));
            cout << "pivot generated" << endl;
            dijkstraSearch(&t, i);
        }
        writeToFile({0,1,2,3,4,5,6,7,8,9});
    }
}


void DifferentialHeuristic::furthestPlacement() {
    //cout << "furthest Placement" << endl;
    std::string name = "../heuristic_furthest.txt";
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
        std::vector<State> t, h;
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
                if (env.allStates[f->second].gcost == env.inf) {
                    isfilled = true;
                }
            }
        } while (isfilled);
        t.push_back(State(tempstate));
        dijkstraSearch(&t, 29);
        for (unsigned int j = 0; j < allHash.size(); ++j) {
            allHash[j][29] = env.inf;
        }
        t.clear();
        cout << "random done" << endl;
        //tempstate = lastState.getState();
        for (int i = 0; i < 10; i++) {
            t.push_back(lastState);
            dijkstraSearch(&t, i);
            cout << i << endl;
            for (unsigned int j = 0; j < allHash.size(); ++j) {
                allHash[j][i] = env.inf;
            }
        }
        for (int i = 0; i < 10; ++i) {
            h.clear();
            h.push_back(t[i]);
            cout << t[i].getState()[0] << ' ' << t[i].getState()[1] << ' ' << t[i].getState()[2] << endl;
            dijkstraSearch(&h, i);
            cout << "heuristic" << i << "built" << endl;
        }
        writeToFile({0,1,2,3,4,5,6,7,8,9});
    }
}

bool sortdesc(const pair<float,int> &a,
              const pair<float,int> &b) {
    return (a.first > b.first);
}

void DifferentialHeuristic::optimizedPlacement() {
    std::string name = "../heuristic_optimized.txt";
    struct stat buffer;
    std::vector<int> tempstate;
    std::vector<State> t, samples;
    unsigned long long t1, t2;
    std::vector<State> h;
    std::vector<std::pair<float, int>> compare;
    for (int i = 0; i < 30; ++i) {
        compare.push_back(std::make_pair(0,i));
    }
    float first;
    t.clear();
    unsigned long long hashtemp;
    bool isfilled = false;
    
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
                if (env.allStates[f->second].gcost == env.inf) {
                    isfilled = true;
                }
            }
        } while (isfilled);
        t.push_back(State(tempstate));
        dijkstraSearch(&t, 29);
        for (unsigned int j = 0; j < allHash.size(); ++j) {
            allHash[j][29] = env.inf;
        }
        t.clear();
        cout << "random done" << endl;
        //tempstate = lastState.getState();
        for (int i = 0; i < 30; i++) {
            t.push_back(lastState);
            dijkstraSearch(&t, i);
            cout << i << endl;
            for (unsigned int j = 0; j < allHash.size(); ++j) {
                allHash[j][i] = env.inf;
            }
        }
        for (int i = 0; i < 30; ++i) {
            h.clear();
            h.push_back(t[i]);
            cout << t[i].getState()[0] << ' ' << t[i].getState()[1] << ' ' << t[i].getState()[2] << endl;
            dijkstraSearch(&h, i);
            cout << "heuristic" << i << "built" << endl;
        }
        
        // generate samples
        for (int i = 0; i < 100; i++) {
            tempstate.clear();
            do {
                srand( static_cast<unsigned int>(time(NULL)));
                tempstate.clear();
                isfilled = false;
                tempstate.push_back(rand()%mapSize[0]);
                tempstate.push_back(rand()%mapSize[1]);
                tempstate.push_back(rand()%mapSize[2]);
                hashtemp = env.getStateHash(State(tempstate));
                std::map<unsigned long long, int>::iterator f =
                env.hashtable.find(hashtemp);
                if (f != env.hashtable.end()) {
                    if (env.allStates[f->second].gcost == env.inf) {
                        isfilled = true;
                    }
                }
                for (int j = 0; j < (int)samples.size(); ++j) { // ensure no same point is generated
                    if (tempstate[0]==samples[j].getState()[0] && tempstate[1]==samples[j].getState()[1]
                        && tempstate[2]==samples[j].getState()[2]){
                        isfilled = true;
                        break;
                    }
                }
            } while (isfilled);
            samples.push_back(State(tempstate));
        }
        
        // calculate gains comparing to the first heuristic.
        for (int i = 0; i < 50; i++) {
            t1 = env.getStateHash(samples[i*2]);
            t2 = env.getStateHash(samples[i*2+1]);
            first = octileHeuristic(allHash[t1][0], allHash[t2][0]);
            for (int j = 0; j < 30; ++j) {
                compare[j].first += fmax(0, (fabs(allHash[t1][j] - allHash[t2][j]) - first));
            }
        }
        
        // measure the best 10 heuristics by comparing heuristic gains.
        std::vector<int> ind;
        std::sort(compare.begin(), compare.end(), sortdesc);
        for (int i = 0; i < 10; ++i) {
            ind.push_back(compare[i].second);
        }
        std::sort(ind.begin(), ind.end());
        writeToFile(ind);
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

float DifferentialHeuristic::octileHeuristic(State st) {
    float x,y,dmin,dmax;
    tempstate1 = st.getState();
    tempstate2 = env.getGoal().getState();
    x = abs(tempstate1[0] - tempstate2[0]);
    y = abs(tempstate1[1] - tempstate2[1]);
    dmin = fmin(x, y);
    dmax = fmax(x, y);
    return (sqrt(2)-1)*dmin + dmax;
}


float DifferentialHeuristic::voxelHeuristic(State st1, State st2) {
    float x,y,z,dmin,dmax,dmid;
    tempstate1 = st1.getState();
    tempstate2 = st2.getState();
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

float DifferentialHeuristic::voxelHeuristic(State st) {
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

float DifferentialHeuristic::HCost(State st) {
    //return 0;
    float val = voxelHeuristic(st);
    //float val1 = singleInconsistentLookup(st);
    //float val1 = singleConsistentLookup(st);
    float val1 = multipleLookup(st);
    if (val > val1) return val; 
    else return val1;
}


#endif /* differentialHeuristic_h */
