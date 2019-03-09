//
//  Header.h
//  search-C
//
//  Created by Yourui Guo on 2019-03-08.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#ifndef Header_h
#define Header_h

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
#include "stp_env.hpp"

using namespace std;

class Pattern
{
private:
    State pattern;
    int numPattern;
    
public:
    Pattern(std::vector<int> v, State state);
    int getNumPattern();
    State getPattern();
    ~Pattern();
};

Pattern::Pattern(std::vector<int> v, State state) {
    std::vector<int> s = state.getState();
    std::vector<int>::size_type j = 0;
    for (std::vector<int>::size_type i = 0; i < s.size(); ++i) {
        if (s[i] != v[j]) {
            s[i] = -1;
        }
        else{
            j++;
        }
    }
    pattern = State(s);
    numPattern = (int)v.size(); // number of tiles in pattern
    
}

int Pattern::getNumPattern(){
    return numPattern;
}

State Pattern::getPattern(){
    return pattern;
}

class Heuristic
{
private:
    Stp_env env = Stp_env();
    unsigned int size;
    std::map<int, int> ranks; //ranking, heuristic
    int numExpandNode;
    
public:
    Heuristic();
    Heuristic(State s, State g);
    ~Heuristic();
    int HCost(State state);
    int ManhattanDistance(State state);
    void patternDatabase(Pattern p);
    void BFS(State start);
    int ranking(State p);
    State unranking();
    int factorial(int n);
    Stp_env getEnv();
};

Heuristic::Heuristic(){}
Heuristic::~Heuristic(){}
Heuristic::Heuristic(State s, State g) {
    env = Stp_env(s, g);
    size = env.getSize();
}

Stp_env Heuristic::getEnv() {
    return env;
}

int Heuristic::HCost(State state) {
    return ManhattanDistance(state);
}

int Heuristic::ManhattanDistance(State state) {
    int hcost = 0;
    int empty = env.getEmpty(state);
    for (std::vector<int>::size_type i = 0; (unsigned int)i < size*size; ++i) {
        if ((int)i == empty) {
            continue;
        }
        int r = state.getState()[i] / (int)size;
        int c = state.getState()[i] % (int)size;
        hcost += std::abs((int)(r - i/(int)size));
        hcost += std::abs((int)(c - i%(int)size));
    }
    return hcost;
}


void Heuristic::patternDatabase(Pattern p) {
    numExpandNode = factorial(size*size) /
    factorial(size*size-p.getNumPattern());
    BFS(p.getPattern());
}


void Heuristic::BFS(State start) {
    std::queue<State> Q;
    int depth = 0, new_states = 0;
    Q.push(start);
    int s_rank = ranking(start);
    ranks[s_rank] = 0;
    while(!Q.empty()) {
        State temp = Q.front();
        int temp_rank = ranking(temp);
        Q.pop();
        if (ranks[temp_rank] > depth){
            depth = ranks[temp_rank];
            std::cout << "new_states:" << new_states << endl;
            new_states = 0;
        }
        else{
            new_states += 1;
        }
        std::vector<Action> actions = env.getActions(temp);
        for (std::vector<Action>::size_type i = 0; i < actions.size(); ++i) {
            Action action = actions[i];
            int t = env.applyActionCopy(action, temp);
            State st = env.allStates[t].getState();
            int next_rank = ranking(st);
            std::map<int, int>::iterator f = ranks.find(next_rank);
            if (f == ranks.end()) {
                ranks[next_rank] = ranks[temp_rank]+1;
                Q.push(st);
            }
        }
    }
}

int Heuristic::ranking(State p) {
    return 0;
    
}

int Heuristic::factorial(int n) {
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

#endif /* Header_h */
