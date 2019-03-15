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
    std::vector<int> ranks, ranks1, ranks2, ranks3; //ranking, heuristic
    unsigned long long numExpandNode;
    std::vector<int> pattern;
    std::vector<int> pattern1;
    std::vector<int> pattern2;
    std::vector<int> pattern3;
    int isBuilt;
    bool isDeltaEnabled;
    bool isMinCompressed;
    bool isMD;
    int factor, rk4;
    Pattern pattern_instance;
    std::vector<int> ts, dual, hstate, unrankstate, patternstate, patterntemp;
    std::vector<unsigned long long> unrank;
    unsigned long long r, f, a, b, rk1, rk2, rk3;
    std::vector<int> unstate;
    State temprank;
    double delta;
public:
    Heuristic();
    Heuristic(State s, State g);
    ~Heuristic();
    int HCost(State state);
    int ManhattanDistance(State state);
    void patternDatabase(State p);
    void minCompression(int rank);
    void BFS(State start);
    unsigned long long lexicographicalRanking(State p, std::vector<int> pat);
    unsigned long long linearTimeRanking(State p);
    unsigned long long rank1(int n, std::vector<int> *state, std::vector<int> *dual);
    unsigned long long factorial(unsigned long long n);
    Stp_env getEnv();
    void setIsDeltaEnabled(bool i);
    void setIsMinCompressed(bool i);
    void setIsMD(bool i);
    State getPatternState(State state, std::vector<int> p);
    State unranking(unsigned long long r, std::vector<int> pat);
    void statistics();
    void writeToFile();
};

Heuristic::Heuristic(){}
Heuristic::~Heuristic(){}
Heuristic::Heuristic(State s, State g) {
    env = Stp_env(s, g);
    size = env.getSize();
    pattern1 = {0,1,2,3,4,5,6,7};
    pattern2 = {0,8,9,12,13};
    pattern3 = {0,10,11,14,15};
    pattern = pattern3;
    isBuilt = 0;
    factor = 2;
    isDeltaEnabled = false;
    isMinCompressed = false;
    isMD = false;
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


State Heuristic::getPatternState(State state, std::vector<int> pat) {
    patternstate = state.getState();
    patterntemp.clear();
    patterntemp.resize(size*size, -1);
    std::vector<int>::size_type i,j = 0;
    for (i = 0; i < patternstate.size(); ++i) {
        for (j = 0; j < pat.size(); j++) {
            if (patternstate[i] == pat[j]) patterntemp[i] = pat[j];
        }
    }
    return State(patterntemp);
}

void Heuristic::writeToFile() {
    State p = pattern_instance.getPattern();
    std::string name = "./heuristic_delta3.txt";
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
    cout<< "file written" << endl;
    cout << delta << endl;
}

int Heuristic::HCost(State state) {
    //cout << "here" << endl;
    if (isMD){
        return ManhattanDistance(state);
    }
    else{
        if (!isBuilt) {
            //pattern_instance = Pattern(pattern, env.getGoal());
            patternDatabase(pattern_instance.getPattern());
            statistics();
        }

        temprank = getPatternState(state, pattern1);
        rk1 = lexicographicalRanking(temprank, pattern1);

        temprank = getPatternState(state, pattern2);
        rk2 = lexicographicalRanking(temprank, pattern2);

        temprank = getPatternState(state, pattern3);
        rk3 = lexicographicalRanking(temprank, pattern3);

        rk4 = ManhattanDistance(state);

        int max = ranks1[rk1];
        if (ranks2[rk2] > max) {
            max = ranks2[rk2];
        }
        if (ranks3[rk3] > max){
            max = ranks3[rk3];
        }
        if (rk4 > max){
            max = rk4;
        }
        //cout << ranks1[rk1] << ' ' << ranks2[rk2] << ' ' << ranks3[rk3] << ' ' << rk4 << endl;
        return max;
    }
}

void Heuristic::statistics() {
    std::vector<int>::size_type i;
    double avg=0;
    int min, index;

    avg = 0;
    for (i = 1; i < ranks1.size()+1; ++i) {
        index = (i-1)/2;
        min = ranks1[index*2];
        if (ranks1[index*2+1] < min) ranks1[index*2] = ranks1[index*2+1];
        else ranks1[index*2+1] = ranks1[index*2];
        avg = ((i-1)*1.0/i)*avg + (1.0/i)*ranks1[i-1];
    }
    cout << "pdb1 average: " << avg << endl;

    avg = 0;
    for (i = 1; i < ranks2.size()+1; ++i) {
        index = (i-1)/2;
        min = ranks2[index*2];
        if (ranks2[index*2+1] < min) ranks2[index*2] = ranks2[index*2+1];
        else ranks2[index*2+1] = ranks2[index*2];
        avg = ((i-1)*1.0/i)*avg + (1.0/i)*ranks2[i-1];
    }
    cout << "pdb2 average: " << avg << endl;
    
    avg=0;
    for (i = 1; i < ranks3.size()+1; ++i) {
        index = (i-1)/2;
        min = ranks3[index*2];
        if (ranks3[index*2+1] < min) ranks3[index*2] = ranks3[index*2+1];
        else ranks3[index*2+1] = ranks3[index*2];
        avg = ((i-1)*1.0/i)*avg + (1.0/i)*ranks3[i-1];
    }
    cout << "pdb3 average: " << avg << endl;
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
    std::string name1 = "./heuristic_delta1.txt";
    std::string name2 = "./heuristic_delta2.txt";
    std::string name3 = "./heuristic_delta3.txt";
    if (stat (name1.c_str(), &buffer) == 0) { // readfile
        ifstream f(name1);
        //assert(f.is_open());
        std::copy(std::istream_iterator<int>(f), std::istream_iterator<int>(),
                  std::back_inserter(ranks1));
        f.close();
        cout << "PDB1 built" << endl;
    }
    if (stat (name2.c_str(), &buffer) == 0) { // readfile
        ifstream f(name2);
        //assert(f.is_open());
        std::copy(std::istream_iterator<int>(f), std::istream_iterator<int>(),
                  std::back_inserter(ranks2));
        f.close();
        cout << "PDB2 built" << endl;
    }
    if (stat (name3.c_str(), &buffer) == 0) { // readfile
        ifstream f(name3);
        //assert(f.is_open());
        std::copy(std::istream_iterator<int>(f), std::istream_iterator<int>(),
                  std::back_inserter(ranks3));
        f.close();
        cout << "PDB3 built" << endl;
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
    int count = 0;
    delta = 0;


    temp_rank = lexicographicalRanking(start, pattern);
    ranks[temp_rank] = 0;
    Q.push(temp_rank);
    
    while(!Q.empty()) {
        temp_rank = Q.front();
        temp = unranking(temp_rank, pattern);
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
            next_rank = lexicographicalRanking(temp, pattern);
            if (ranks[next_rank] == -1) {
                ranks[next_rank] = ranks[temp_rank]+1;
                Q.push(next_rank);
            }
            env.undoAction(action, &temp);
        }
        ranks[temp_rank] -= ManhattanDistance(temp);
        if (ranks[temp_rank] < 0){
            ranks[temp_rank] = 0;
        }
    }
    std::cout << "depth: " << depth << " new_states: " << new_states << endl;
}


unsigned long long Heuristic::lexicographicalRanking(State p, std::vector<int> pat) {
    r = 0;
    int i, j, k;
    //int n = pat.size();
    ts = p.getState();
    dual.clear();
    dual.resize(size*size, -1);
    int count = 0;
    
    for (i = 0; i < (int)ts.size(); i++) {
        if (ts[i] != -1) {
            dual[ts[i]] = (int)i;
        }
    }
    
    for (i = 0; i < (int)dual.size(); i++) {
        if (dual[i] != -1){
            count++;
            k = dual[i];
            for (j = 0; j < i; j++) {
                if ((dual[j] != -1) && (dual[j] < dual[i])) {
                    k -= 1;
                }
            }
            f = factorial(size*size-count);
            r += k*f;
        }
    }
    f = factorial(size*size-pat.size());
    r = r/f;
    return r;
}

State Heuristic::unranking(unsigned long long r, std::vector<int> pat) {
    //State st;
    unstate.clear();
    unstate.resize(16, -1);
    int i, j;
    int nums = pat.size();
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
        unstate[unrank[i]] = pat[i];
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
