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
#include <sys/stat.h>
#include <unistd.h>
#include "stp_env.hpp"

using namespace std;

class Pattern
{
private:
    State pattern;
    int numPattern;
    
public:
    Pattern();
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
Pattern::Pattern(){}
Pattern::~Pattern(){}

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
    std::vector<int> ranks; //ranking, heuristic
    unsigned long long numExpandNode;
    std::vector<int> pattern;
    std::vector<int> pattern1;
    std::vector<int> pattern2;
    std::vector<int> pattern3;
    int isBuilt;
    bool isDeltaEnabled;
    bool isMinCompressed;
    bool isMD;
    int factor;
    Pattern pattern_instance;
    std::vector<int> ts, dual, hstate, unrankstate;
    std::vector<unsigned long long> unrank;
    unsigned long long r, f, a, b;
    std::vector<int> unstate;
public:
    Heuristic();
    Heuristic(State s, State g);
    ~Heuristic();
    int HCost(State state);
    int ManhattanDistance(State state);
    void patternDatabase(State p);
    void minCompression(int rank);
    void BFS(State start);
    unsigned long long lexicographicalRanking(State p);
    unsigned long long linearTimeRanking(State p);
    unsigned long long rank1(int n, std::vector<int> *state, std::vector<int> *dual);
    unsigned long long factorial(unsigned long long n);
    Stp_env getEnv();
    void setIsDeltaEnabled(bool i);
    void setIsMinCompressed(bool i);
    void setIsMD(bool i);
    State getPatternState(State st);
    State unranking(unsigned long long r);
};

Heuristic::Heuristic(){}
Heuristic::~Heuristic(){}
Heuristic::Heuristic(State s, State g) {
    env = Stp_env(s, g);
    size = env.getSize();
    pattern1 = {0,1,2,3,4,5,6,7};
    pattern2 = {0,8,9,12,13};
    pattern3 = {0,10,11,14,15};
    pattern = pattern1;
    isBuilt = 0;
    factor = 2;
    isDeltaEnabled = false;
    isMinCompressed = false;
    isMD = true;
    pattern_instance = Pattern(pattern, env.getGoal());
    unrank.resize(pattern_instance.getNumPattern());
    unstate.resize(16, -1);
    //unrankstate.resize(size*size, -1);
}

void Heuristic::setIsDeltaEnabled(bool i) {
    isDeltaEnabled = i;
}

void Heuristic::setIsMinCompressed(bool i) {
    isMinCompressed = i;
}

void Heuristic::setIsMD(bool i) {
    isMD = i;
}

Stp_env Heuristic::getEnv() {
    return env;
}


State Heuristic::getPatternState(State state) {
    std::vector<int> s = state.getState();
    std::vector<int> t(size*size, -1);
    std::vector<int>::size_type j = 0;
    for (std::vector<int>::size_type i = 0; i < s.size(); ++i) {
        for (j = 0; j < (unsigned int)pattern_instance.getNumPattern(); j++) {
            if (s[i] == pattern[j]) t[i] = s[i];
        }
    }
    return State(t);
}

int Heuristic::HCost(State state) {
    if (isMD){
        return ManhattanDistance(state);
    }
    else{
        if (!isBuilt) {
            //pattern_instance = Pattern(pattern, env.getGoal());
            patternDatabase(pattern_instance.getPattern());
        }
        State temp = getPatternState(state);
        unsigned long long r = lexicographicalRanking(temp);
        return ranks[r];
    }
}

int Heuristic::ManhattanDistance(State s) {
    int hcost = 0;
    int empty = env.getEmpty(s);
    std::vector<int> state = s.getState();
    for (std::vector<int>::size_type i = 0; (unsigned int)i < state.size(); ++i) {
        if ((int)i == empty) {
            continue;
        }
        if (state[i] != -1) {
            int r = state[i] / (int)size;
            int c = state[i] % (int)size;
            hcost += std::abs((int)(r - i/(int)size));
            hcost += std::abs((int)(c - i%(int)size));
        }
    }
    return hcost;
}




void Heuristic::patternDatabase(State p) {
    struct stat buffer;
    std::string name = "/Users/margaret/Documents/cmput652/search-C1/heuristic.txt";
    if (stat (name.c_str(), &buffer) == 0) { // readfile
        ifstream f(name);
        assert(f.is_open());
        std::copy(std::istream_iterator<int>(f), std::istream_iterator<int>(),
                  std::back_inserter(ranks));
        f.close();
    }
    else {
        unsigned long long a = factorial(size*size);
        unsigned long long b = factorial(size*size-pattern_instance.getNumPattern());
        numExpandNode = a/b;
        ranks.reserve(numExpandNode);
        for (std::vector<int>::size_type i = 0; i < numExpandNode; i++) {
            ranks.push_back(-1);
        }
        BFS(p);
        
        //write to file
        ofstream f(name);
        if (f.is_open()) {
            for (int i = 0; i < numExpandNode; i++) {
                f << ranks[i] << ' ';
            }
        }
        f.close();
        
    }
    
    isBuilt = 1;
}



void Heuristic::BFS(State start) {
    std::queue<unsigned long long> Q;
    int depth = 0, new_states = 0;
    State temp, st;
    Action action = Action(-1);
    //int a, c;
    unsigned long long temp_rank, next_rank;
    std::vector<Action> actions;
    
    temp_rank = lexicographicalRanking(start);
    ranks[temp_rank] = 0;
    Q.push(temp_rank);
    
    while(!Q.empty()) {
        temp_rank = Q.front();
        temp = unranking(temp_rank);
        Q.pop();
        if (ranks[temp_rank] > depth){
            std::cout << "depth: " << depth << " new_states: " << new_states << endl;
            new_states = 1;
            depth++;
        }
        else{
            new_states += 1;
        }
        env.getActions(temp, &actions);
        for (std::vector<Action>::size_type i = 0; i < actions.size(); ++i) {
            action = actions[i];
            env.applyAction(action, &temp);
            next_rank = lexicographicalRanking(temp);
            if (ranks[next_rank] == -1) {
                ranks[next_rank] = ranks[temp_rank]+1;
                /*
                 if (isDeltaEnabled) {
                 ranks[next_rank] -= ManhattanDistance(st);
                 }
                 if (isMinCompressed){
                 minCompression(next_rank);
                 }*/
                Q.push(next_rank);
            }
            env.undoAction(action, &temp);
        }
    }
    std::cout << "depth: " << depth << " new_states: " << new_states << endl;
}


unsigned long long Heuristic::lexicographicalRanking(State p) {
    r = 0;
    int i, j;
    int nums = (int)pattern.size();
    ts = p.getState();
    dual.resize(size*size, -1);
    int count = 0;
    
    for (i = 0; i <ts.size(); i++) {
        if (ts[i] != -1) {
            dual[ts[i]] = (int)i;
        }
    }
    
    for (i = 0; i < dual.size(); i++) {
        if (dual[i] != -1){
            count++;
            int k = dual[i];
            for (j = 0; j < i; j++) {
                if ((dual[j] != -1) && (dual[j] < dual[i])) {
                    k -= 1;
                }
            }
            f = factorial(size*size-count);
            r += k*f;
        }
    }
    f = factorial(size*size-nums);
    r = r/f;
    return r;
}

State Heuristic::unranking(unsigned long long r) {
    //State st;
    unstate.clear();
    unstate.resize(16, -1);
    int i, j;
    int nums = (int)pattern.size();
    for (i = 0; i < nums-1; i++) {
        a = r%((size*size)-nums+i+1);
        b = r/((size*size)-nums+i+1);
        unrank[nums-i-1] = a;
        r = b;
    }
    unrank[0] = r;
    for (i = nums-2; i >= 0; i--) {
        for (j = i+1; j < nums; j++) {
            if (unrank[i] <= unrank[j]) {
                unrank[j]++;
            }
        }
    }
    for (i = 0; i < nums; i++) {
        unstate[unrank[i]] = pattern[i];
    }
    //st = State(unstate);
    return State(unstate);
}



unsigned long long Heuristic::linearTimeRanking(State p) {
    r = 0;
    int nums = (int)pattern.size();
    std::vector<int> s = p.getState();
    std::vector<int> dual;
    std::vector<int> state_list;
    
    for (std::vector<int>::size_type i = 0; i < s.size(); i++) {
        if (s[i] != -1) {
            state_list.push_back(s[i]);
            dual.push_back((int)i);
        }
    }
    r = rank1(nums, &state_list, &dual);
    return r;
}

unsigned long long Heuristic::rank1(int n, std::vector<int> *state, std::vector<int> *dual) {
    if (n == 1) return 0;
    int s = (*state)[n-1];
    int l = (*dual)[n-1];
    (*state)[n-1] = (*state)[l];
    (*state)[l] = s;
    (*dual)[n-1] = (*dual)[s];
    (*dual)[s] = l;
    unsigned long long f = factorial(n-1);
    return (s*f + rank1(n-1, state, dual));
}


void Heuristic::minCompression(int rank) {
    int adj = ranks[rank/2+1-rank%2];
    if (adj != -1) {
        int r = rank/2;
        if (ranks[2*r] > ranks[2*r+1]) {
            ranks[2*r] = ranks[2*r+1];
        }
    }
}

unsigned long long Heuristic::factorial(unsigned long long n) {
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

#endif /* Header_h */
