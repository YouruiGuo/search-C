//
//  astar.cpp
//  search-C
//
//  Created by Yourui Guo on 2019-03-08.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iterator>
#include <unordered_map>
#include <string.h>
#include <deque>
#include "heuristic.hpp"
#include "stp_env.hpp"


class Heap{
private:
    std::deque<int> queue;
    std::vector<StateInfo> *allStates;
public:
    Heap();
    Heap(std::vector<StateInfo> *a);
    ~Heap();
    void heap_push(int index, int hcost);
    int heap_pop();
    void sift_up(int index);
    void sift_down(int index);
    int min_child(int index);
    void heap_swap(int i, int j);
    int getQueueLength();
};


Heap::Heap(std::vector<StateInfo> *a){
    queue = {};
    allStates = a;
}
Heap::Heap(){}
Heap::~Heap(){}

int Heap::getQueueLength() {
    return (int)queue.size();
}

void Heap::heap_push(int index, int hcost) {
    queue.push_back(index);
    (*allStates)[index].open_id = (int)queue.size()-1;
    (*allStates)[index].hcost = hcost;
    sift_up((*allStates)[index].open_id);
}

int Heap::heap_pop() {
    int result = queue.front();
    queue.front() = queue.back();
    queue.pop_back();
    sift_down(0);
    return result;
}

void Heap::sift_up(int index) {
    
    while (((index-1)/2 >= 0)) {
        if (index == 0) {
            break;
        }
        int a = (*allStates)[queue[index]].gcost+
        (*allStates)[queue[index]].hcost;
        int b = (*allStates)[queue[(index-1)/2]].gcost+
        (*allStates)[queue[(index-1)/2]].hcost;
        if (a == b) {
            if ((*allStates)[queue[index]].gcost <
                (*allStates)[queue[(index-1)/2]].gcost) {
                heap_swap(index, (index-1)/2);
            }
        }
        if (a < b) {
            heap_swap(index, (index-1)/2);
        }
        index = (index-1)/2;
    }
}

void Heap::sift_down(int index) {
    while (index*2+1 < queue.size()) {
        int mc = min_child(index);
        int a = (*allStates)[queue[index]].gcost+ // parent
        (*allStates)[queue[index]].hcost;
        int b = (*allStates)[queue[mc]].gcost+ // child
        (*allStates)[queue[mc]].hcost;
        if (a > b) {
            heap_swap(index, mc);
        }
        index = mc;
    }
}

int Heap::min_child(int index) {
    if (index*2+2 >= queue.size()) {
        return index*2+1;
    }
    int a = (*allStates)[queue[index*2+1]].gcost+ // left child
    (*allStates)[queue[index*2+1]].hcost;
    int b = (*allStates)[queue[index*2+2]].gcost+ // fight child
    (*allStates)[queue[index*2+2]].hcost;
    if (a == b) {
        if ((*allStates)[queue[index*2+1]].gcost >
            (*allStates)[queue[index*2+2]].gcost) {
            return index*2+1;
        }
        else return index*2+2;
    }
    if (a < b) {
        return index*2+1;
    }
    return index*2+2;
}

void Heap::heap_swap(int i, int j) {
    int temp = queue[i];
    queue[i] = queue[j];
    queue[j] = temp;
    (*allStates)[queue[i]].open_id = i;
    (*allStates)[queue[j]].open_id = j;
}


class Astar {
private:
    Heuristic heu;
    Stp_env env;
    int start_index;
    State start;
    State goal;
    Heap openlist;
    
public:
    int expanded;
    int updated;
    int max_open;
    std::queue<State> *path;
    Astar(State s, State g);
    ~Astar();
    bool search();
    std::queue<State> *getPath();
};

Astar::Astar(State s, State g) {
    start = s;
    goal = g;
    heu = Heuristic(s, g);
    env = heu.getEnv();
    unsigned long long st = env.getStateHash(s);
    start_index = env.getHashIndex(st);
    env.allStates[start_index].gcost = 0;
    env.allStates[start_index].hcost = heu.HCost(s);
    std::vector<StateInfo> allStates = env.allStates;
    openlist = Heap(&env.allStates);
    max_open = 0;
    expanded = 0;
    updated = 0;
}
Astar::~Astar(){}

bool Astar::search() {
    State start = env.getStart();
    openlist.heap_push(start_index, heu.HCost(start));
    while (openlist.getQueueLength() != 0) {
        int index = openlist.heap_pop();
        State st = env.allStates[index].getState();
        env.allStates[index].open_id = -1;
        expanded += 1;
        if (env.isSuccess(st)) {
            //path = getPath();
            return true;
        }
        std::vector<Action> actions = env.getActions(st);
        for (std::vector<Action>::size_type i = 0; i < actions.size(); i++) {
            Action action = actions[i];
            int next = env.applyActionCopy(action, st);
            State next_state = env.allStates[next].getState();
            int open_id = env.allStates[next].open_id;
            if (open_id == -1) {
                continue;
            }
            else if (open_id > -1) {
                int newcost = env.allStates[index].gcost+
                                env.cost(st, next_state);
                if (newcost < env.allStates[next].gcost) {
                    updated += 1;
                    env.allStates[next].gcost = newcost;
                    env.allStates[next].parent = index;
                    openlist.sift_up(env.allStates[next].open_id);
                }
            }
            else {
                env.allStates[next].gcost = env.allStates[index].gcost+
                                            env.cost(st, next_state);
                env.allStates[next].parent = index;
                openlist.heap_push(next, heu.HCost(next_state));
                max_open += 1;
            }
        }
    }
    return false;
}


std::queue<State> *Astar::getPath() {
    path->push(env.getGoal());
    unsigned long long v = env.getStateHash(env.getGoal());
    StateInfo state = env.allStates[env.getHashIndex(v)];
    while (state.parent != -1) {
        path->push(env.allStates[state.parent].getState());
        state = env.allStates[state.parent];
    }
    return path;
}

State loadstpFile(int index){
    
    std::vector<State> instances;
    std::string df = "/Users/margaret/Documents/cmput652/korf100.txt";
    std::ifstream infile(df);
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::vector<string> tokens{std::istream_iterator<string>(iss),
            std::istream_iterator<string>()};
        std::vector<int> data;
        for(vector<string>::const_iterator i = tokens.begin()+1; i != tokens.end(); ++i) {
            // process i
            data.push_back(std::stoi(*i));
        }
        instances.push_back(State(data));
    }
    return instances[index];
}


int main(int argc, char const *argv[])
{
    std::vector<int> g = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    State goal = State(g);
    
    int index = 8;
    State benchmark = loadstpFile(index);
    //std::vector<int> s = {1,2,3,7,0,4,5,6,8,9,10,11,12,13,14,15};
    //State benchmark = State(s);
    
    Astar search = Astar(benchmark, goal);
    search.search();
    //std::queue<State> *path = search.getPath();
    cout << "expanded: " << search.expanded << " updated: " << search.updated << " max open: " << search.max_open << endl;
    return 0;
}
