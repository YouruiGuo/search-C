//
//  astar.cpp
//  search-C
//
//  Created by Yourui Guo on 2019-03-08.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

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
#include "grid_env.hpp"
//#include "differentialHeuristic.hpp"

typedef std::pair<float, int> weight_vertex_pair;

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
    Heuristic heu = Heuristic();
    //Voxel_env env = Voxel_env();
    Grid_env env = Grid_env();
    int start_index;
    State start;
    State goal;
    Heap openlist;
    std::vector<State> succes;
    std::vector<int> parent, cur, act, isuccess_t, isuccess_g;
    
public:
    float totalcost;
    int expanded;
    int updated;
    int max_open;
    std::vector<State> path;
    Astar(State s, State g);
    ~Astar();
    bool canonicalAstar();
    bool boundedJPS();
    bool canonicalDijkstra();
    bool canonicalWeightedAstar();
    bool CanonicalOrdering(int child, int parent, int depth, float cost);
    std::vector<State> getPath();
    Action prevAction(StateInfo st);
    Action prevAction(int child, int par);
};

Astar::Astar(State s, State g) {
    heu = Heuristic(s, g);
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

Action Astar::prevAction(StateInfo st) {
    bool isstart = true;
    isuccess_t = st.getState().getState();
    isuccess_g = start.getState();
    for (unsigned int i=0; i<2; i++) {
        if (isuccess_t[i] != isuccess_g[i]) {
            isstart = false;
        }
    }
    if (isstart) {
        return Action({10, 10});
    }
    act.clear();
    parent = env.allStates[st.parent].getState().getState();
    cur = st.getState().getState();
    for (int i = 0; i < parent.size(); i++) {
        act.push_back(cur[i]-parent[i]);
    }
    return Action({act});
}

Action Astar::prevAction(int child, int par) {
    if (par == -1) {
        return Action({10, 10});
    }
    act.clear();
    cur = env.allStates[child].getState().getState();
    parent = env.allStates[par].getState().getState();
    for (int i = 0; i < parent.size(); i++) {
        act.push_back(cur[i]-parent[i]);
    }
    return Action({act});
}

bool Astar::canonicalAstar() {
    std::vector<Action> actions;
    int next, open_id, index;
    float newcost;
    Action action = Action({0});
    Action prevaction = Action({10, 10});
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
        prevaction = prevAction(env.allStates[index]);
        env.getActions(st, prevaction, &actions);
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


bool Astar::boundedJPS() {
    bool result = false;
    std::vector<State> paths;
    int index;
    int depth = 4;
    State start = env.getStart();
    openlist.heap_push(start_index, heu.HCost(start));
    max_open++;
    while (openlist.getQueueLength() != 0) {
        index = openlist.heap_pop();
        //env.allStates[index].open_id = -1;
        expanded++;
        result = CanonicalOrdering(index, env.allStates[index].parent, depth, env.allStates[index].gcost);
        if (result) {
            //paths = getPath();
            return true;
        }
    }
    return false;
    //return true;
}

bool Astar::CanonicalOrdering(int child, int parent, int depth, float cost) {
    std::vector<Action> actions;
    StateInfo st = env.allStates[child];
    int open_id = env.allStates[child].open_id;
    Action prevaction = prevAction(child, parent);
    
    if (env.isSuccess(st.getState())) {
        totalcost = cost;
        return true;
    }
    if (open_id == -1) {
        if (st.gcost > cost) {
            env.allStates[child].gcost = cost;
            env.allStates[child].parent = parent;
            updated++;
        }
        return false;
    }
    if (depth == 0 || env.isJumpPoint(st.getState(), prevaction)) {
        env.allStates[child].gcost = cost;
        env.allStates[child].parent = parent;
        if (open_id == -1 || open_id == -10) {
            openlist.heap_push(child, heu.HCost(st.getState()));
            return false;
        }
    }
    max_open++;
    env.allStates[child].gcost = cost;
    env.allStates[child].parent = parent;
    env.allStates[child].open_id = -1;
    env.getActions(st.getState(), prevaction, &actions);
    for (std::vector<Action>::size_type i = 0; i < actions.size(); i++) {
        int next = env.applyActionCopy(actions[i], st.getState());
        StateInfo next_state = env.allStates[next];
        float gcost = env.cost(next_state.getState(), st.getState());
        bool p = CanonicalOrdering(next, child, depth-1, cost+gcost);
        if (p) {
            return true;
        }
    }
    
    return false;
}

bool Astar::canonicalDijkstra(){
    int next, index, open_id;
    StateInfo parentstate, currentstate;
    float next_gcost, gcost;
    std::vector<Action> actions;
    bool isjp = false;
    Action action = Action({0});
    Action prevaction = Action({10, 10}), diprevaction = Action({10, 10});
    
    std::priority_queue<weight_vertex_pair,
    std::vector<weight_vertex_pair>,
    std::greater<weight_vertex_pair>> Q;
    
    expanded = 0;
    State start = env.getStart();
    openlist.heap_push(start_index, env.allStates[start_index].gcost);
    while (openlist.getQueueLength() != 0) {
        index = openlist.heap_pop();
        Q.push(std::make_pair(0, index));
        while (!Q.empty()) { // dijkstra's algorithm
            gcost = Q.top().first;
            index = Q.top().second;
            Q.pop(); // pop the top element in queue
            State st = env.allStates[index].getState();
            env.allStates[index].open_id = -1; // add to closed list
            expanded++;
            
            parentstate = env.allStates[index];
            prevaction = prevAction(parentstate);
            // get all possible actions / successors
            env.getActions(parentstate.getState(), prevaction, &actions);
            
            for (unsigned int i = 0; i < actions.size(); i++) {
                action = actions[i];
                next = env.applyActionCopy(action, st);
                currentstate = env.allStates[next];
                open_id = env.allStates[next].open_id;
                next_gcost = env.cost(currentstate.getState(), parentstate.getState());
                
                isjp = env.isJumpPoint(currentstate.getState(), action);
                
                if (open_id == -1) { // closed list
                    if (currentstate.gcost > parentstate.gcost + next_gcost) {
                        // update the g-cost
                        env.allStates[next].gcost = next_gcost + env.allStates[index].gcost;
                        env.allStates[next].parent = index;
                        updated++;
                    }
                    continue;
                }
                
                if (isjp) {
                    env.allStates[next].gcost = next_gcost + env.allStates[index].gcost;
                    env.allStates[next].parent = index;
                    if (env.allStates[next].open_id == -10 || env.allStates[next].open_id == -1) {
                        openlist.heap_push(next, env.allStates[next].gcost);
                    }
                    else {
                        openlist.sift_up(env.allStates[next].open_id);
                    }
                }
                else {
                    // generate new state
                    env.allStates[next].gcost = next_gcost + env.allStates[index].gcost;
                    env.allStates[next].parent = index;
                    Q.push(std::make_pair(env.allStates[next].gcost, next));
                }
            }
        }
    }
    return true;
}

bool Astar::canonicalWeightedAstar(){
    
    
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
    std::cout << pathcost << std::endl;
    return path;
}

void load3dfile(int index, std::vector<int> *s, std::vector<int> *g){
    s->clear();
    g->clear();
    std::vector<State> instances;
    std::string df = "/Users/margaret/Documents/cmput652/searchAlg/data/Simple.3dmap.3dscen";
    std::ifstream infile(df);
    std::string line;
    int count = -1;
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

void loadGridFile(int index, std::vector<int> *s, std::vector<int> *g) {
    s->clear();
    g->clear();
    std::vector<State> instances;
    std::string df = "/Users/margaret/Documents/cmput652/search-C1/asg4/data/brc101d.map.scen";
    std::ifstream infile(df);
    std::string line;
    int count = -1;
    while (std::getline(infile, line)) {
        if (count++ == index) {
            std::istringstream iss(line);
            std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
                std::istream_iterator<std::string>()};
            s->push_back(std::stoi(tokens[4]));
            s->push_back(std::stoi(tokens[5]));
            g->push_back(std::stoi(tokens[6]));
            g->push_back(std::stoi(tokens[7]));
            break;
        }
    }
}

int main(int argc, char const *argv[])
{
    //std::vector<int> g = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    //State goal = State(g);
    std::vector<int> v;
    
    
    for (int i = 0; i < 1570; ++i){
        std::vector<int> s, g;
        //load3dfile(i, &s, &g);
        loadGridFile(i, &s, &g);
        Astar search = Astar(s, g);
        clock_t begin = clock();
        bool result = search.boundedJPS();
        //bool result = search.canonicalAstar();
        //bool result = search.canonicalDijkstra();
        clock_t end = clock();
        //std::queue<State> *path = search.getPath();
        if (result == true){
            std::cout << "num: " << i <<" expanded: " << search.expanded << " updated: " <<
            search.updated << " cost: " << search.totalcost << " max_open: " << search.max_open << " time_elpased: "
            << double(end - begin) / CLOCKS_PER_SEC << std::endl;
        }
        //break;
    }
    
    //State benchmark = loadstpFile(index, &si);
    //std::vector<int> s = {1,2,3,7,0,4,5,6,8,9,10,11,12,13,14,15};
    //State benchmark = State(s);
    
    return 0;
}
