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
#include <cmath>
#include <ctime>
//#include "voxel_env.hpp"
#include "differentialHeuristic.hpp"

class Heap{
private:
    std::deque<int> queue;
    std::vector<StateInfo> *allStates;
public:
    Heap();
    Heap(std::vector<StateInfo> *a);
    ~Heap();
    void heap_push(int index, float hcost);
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

void Heap::heap_push(int index, float hcost) {
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
        float a = (*allStates)[queue[index]].gcost+
        (*allStates)[queue[index]].hcost;
        float b = (*allStates)[queue[(index-1)/2]].gcost+
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
    while (index*2+1 < (int)queue.size()) {
        int mc = min_child(index);
        float a = (*allStates)[queue[index]].gcost+ // parent
        (*allStates)[queue[index]].hcost;
        float b = (*allStates)[queue[mc]].gcost+ // child
        (*allStates)[queue[mc]].hcost;
        if (a > b) {
            heap_swap(index, mc);
        }
        index = mc;
    }
}

int Heap::min_child(int index) {
    if (index*2+2 >= (int)queue.size()) {
        return index*2+1;
    }
    float a = (*allStates)[queue[index*2+1]].gcost+ // left child
    (*allStates)[queue[index*2+1]].hcost;
    float b = (*allStates)[queue[index*2+2]].gcost+ // fight child
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
    DifferentialHeuristic heu = DifferentialHeuristic();
    Voxel_env env = Voxel_env();
    int start_index;
    State start;
    State goal;
    Heap openlist;
    std::vector<State> succes;

public:
    int expanded;
    int updated;
    int max_open;
    std::vector<State> path;
    Astar(State s, State g);
    ~Astar();
    bool search();
    bool searchBPMX();
    std::vector<State> getPath();
};

Astar::Astar(State s, State g) {
    heu = DifferentialHeuristic(s, g);
    env = heu.getEnv();
    start = s;
    goal = g;
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
    std::vector<Action> actions;
    int next, open_id, index;
    float newcost;
    Action action = Action({0});
    State start = env.getStart();

    openlist.heap_push(start_index, heu.HCost(start));
    while (openlist.getQueueLength() != 0) {
        index = openlist.heap_pop();
        State st = env.allStates[index].getState();
        env.allStates[index].open_id = -1;
        expanded++;
        if (env.isSuccess(st)) {
            path = getPath();
            return true;
        }
        env.getActions(st, &actions);
        for (std::vector<Action>::size_type i = 0; i < actions.size(); i++) {
            action = actions[i];
            next = env.applyActionCopy(action, st);
            State next_state = env.allStates[next].getState();
            open_id = env.allStates[next].open_id;
            if (open_id == -1) {
                continue;
            }
            else if (open_id > -1) {
                newcost = env.allStates[index].gcost+
                            env.cost(st, next_state);
                if (newcost < env.allStates[next].gcost) {
                    updated++;
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
                max_open++;
            }
        }
    }
    return false;
}

bool Astar::searchBPMX() {
    Action action = Action({0});
    State next_state, st;
    std::vector<Action> actions;
    float bestH = 0, edgecost, newcost;
    int next, index, open_id;
    // push start to openlist
    State start = env.getStart();
    openlist.heap_push(start_index, heu.HCost(start));

    while (openlist.getQueueLength() != 0) {
        
        // pop best node from openlist
        index = openlist.heap_pop();
        st = env.allStates[index].getState();
        env.allStates[index].open_id = -1;
     
        expanded += 1;
        if (env.isSuccess(st)) {
            path = getPath();
            return true;
        }

        succes.clear();
        bestH = 0; // stores parent h-cost from pathmax
        // generate successors
        env.getActions(st, &actions);
        // cache lookups for later use
        for (std::vector<Action>::size_type i = 0; i < actions.size(); i++) {
            action = actions[i];
            next = env.applyActionCopy(action, st);
            next_state = env.allStates[next].getState();
            succes.push_back(next_state);
            bestH = fmax(heu.HCost(next_state)-env.cost(st, next_state), bestH);
        }
        // store heuristics for current node with bestH.
        if (env.allStates[index].hcost < bestH) {
            env.allStates[index].hcost = bestH;
        }

        for (unsigned int i = 0; i < succes.size(); i++) {
            next_state = succes[i];
            next = env.getHashIndex(env.getStateHash(next_state));
            open_id = env.allStates[next].open_id;
            // generate edge cost
            edgecost = env.cost(st, next_state);

            if (open_id == -1) { // closed list
                if (env.allStates[next].hcost < bestH - edgecost){ // Bidirectional PathMax
                    env.allStates[next].hcost = bestH - edgecost;
                }
                // found a shorter path and re-open
                if (env.allStates[index].gcost + edgecost < env.allStates[next].gcost) { 
                    env.allStates[next].parent = index;
                    env.allStates[next].gcost = env.allStates[index].gcost + edgecost;
                    openlist.heap_push(next, env.allStates[next].hcost);
                }
            }
            else if (open_id > -1) { // open list
                newcost = env.allStates[index].gcost + edgecost;
                if (newcost < env.allStates[next].gcost) {
                    updated += 1;
                    env.allStates[next].gcost = newcost;
                    env.allStates[next].parent = index;
                    openlist.sift_up(env.allStates[next].open_id); // resort openlist
                }
                if (bestH - edgecost > env.allStates[next].hcost) {
                    env.allStates[next].hcost = bestH - edgecost;
                    openlist.sift_down(env.allStates[next].open_id);
                }
            }
            else {
                env.allStates[next].gcost = env.allStates[index].gcost + edgecost;
                env.allStates[next].parent = index;
                openlist.heap_push(next, fmax(heu.HCost(next_state), bestH - edgecost));
                max_open += 1;
            }
        }
    }
    return false;
}


std::vector<State> Astar::getPath() {
    float pathcost = 0;
    path.clear();
    path.push_back(env.getGoal());
    unsigned long long v = env.getStateHash(env.getGoal());
    StateInfo state = env.allStates[env.getHashIndex(v)];
    do {
        //cout << state.getState().getState()[0] << ' ' << state.getState().getState()[1] << ' ' << state.getState().getState()[2] << endl;
        pathcost += env.cost(state.getState(), env.allStates[state.parent].getState());
        path.push_back(env.allStates[state.parent].getState());
        state = env.allStates[state.parent];
    }while (state.parent != -1);
    //cout << pathcost << endl;
    return path;
}

State loadstpFile(int index, std::vector<int> *si){

    std::vector<State> instances;
    std::string df = "/Users/margaret/Documents/cmput652/korf100.txt";
    std::ifstream infile(df);
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
            std::istream_iterator<std::string>()};
        std::vector<int> data;
        for(std::vector<std::string>::const_iterator i = tokens.begin()+1; i != tokens.end(); ++i) {
            // process i
            data.push_back(std::stoi(*i));
        }
        instances.push_back(State(data));
    }
    return instances[index];
}

void load3dfile(int index, std::vector<int> *s, std::vector<int> *g){
    s->clear();
    g->clear();
    std::vector<State> instances;
    std::string df = "./Simple.3dmap.3dscen";
    std::ifstream infile(df);
    std::string line;
    int count = -2;
    while (std::getline(infile, line)) {
        if (count++ == index) {
            std::istringstream iss(line);
            std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
                std::istream_iterator<std::string>()};
            s->push_back(std::stoi(tokens[0]));
            s->push_back(std::stoi(tokens[1]));
            s->push_back(std::stoi(tokens[2]));
            g->push_back(std::stoi(tokens[3]));
            g->push_back(std::stoi(tokens[4]));
            g->push_back(std::stoi(tokens[5]));
            break;
        }
    }
}

int main(int argc, char const *argv[])
{
    //std::vector<int> g = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    //State goal = State(g);
    std::vector<int> v;


    for (int i = 0; i < 1000; ++i){
        std::vector<int> s, g;
        load3dfile(i, &s, &g);
        Astar search = Astar(s, g);
        clock_t begin = clock();
        bool result = search.search();
        //bool result = search.searchBPMX();
        clock_t end = clock();
        //std::queue<State> *path = search.getPath();
        if (result == true){
            std::cout << "num: " << i <<" expanded: " << search.expanded << " updated: " << 
                search.updated << " path_len: " << search.path.size() << " max_open: " << search.max_open << " time_elpased: "
                << double(end - begin) / CLOCKS_PER_SEC << std::endl;
        }
        //break;
    }

    //State benchmark = loadstpFile(index, &si);
    //std::vector<int> s = {1,2,3,7,0,4,5,6,8,9,10,11,12,13,14,15};
    //State benchmark = State(s);

return 0;
}
