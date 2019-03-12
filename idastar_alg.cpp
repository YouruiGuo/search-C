//
//  idastar_alg.cpp
//  search-C
//
//  Created by Yourui Guo on 2019-03-08.
//  Copyright © 2019 Yourui Guo. All rights reserved.
//

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iterator>
#include <unordered_map>
#include <string.h>
#include <stdio.h>
#include "heuristic.hpp"
#include "stp_env.hpp"
using namespace std;

class Idastar_alg {
private:
    int expanded;
    int nextThreshold;
    State start;
    Heuristic heu = Heuristic();
    Stp_env env = Stp_env();
    std::vector<std::vector<Action>> actionVectors;
    
public:
    Idastar_alg();
    Idastar_alg(State s, State g);
    ~Idastar_alg();
    std::vector<Action> search(Action initAct);
    bool DFS(int threshold, int gcost, State *root,
             Action prevAct, std::vector<Action> *path);
    
};


Idastar_alg::Idastar_alg(State s, State g){
    expanded = 0;
    start = s;
    heu = Heuristic(s, g);
    env = heu.getEnv();
    nextThreshold = heu.HCost(s);
    heu.setIsMD(false);
    heu.setIsMinCompressed(false);
    heu.setIsDeltaEnabled(false);
    actionVectors.resize(82);
}

Idastar_alg::~Idastar_alg(){}


std::vector<Action> Idastar_alg::search(Action initAct){
    bool success = false;
    int threshold = nextThreshold;
    int gcost = 0;
    std::vector<Action> path;
    while (!success) {
        success = DFS(threshold, gcost, &start, initAct, &path);
        threshold = nextThreshold;
    }
    return path;
    
}

bool Idastar_alg::DFS(int threshold, int gcost, State* root,
                      Action prevAct, std::vector<Action> *path){
    expanded += 1;
    int fcost = gcost + heu.HCost(*root);
    if (env.isSuccess(*root)) {
        return true;
    }
    if (threshold < fcost) {
        if (nextThreshold > fcost || nextThreshold == threshold) {
            nextThreshold = fcost;
        }
        return false;
    }
    
    env.getActions(*root, &actionVectors[gcost]);
    for (std::vector<Action>::size_type i = 0; i < actionVectors[gcost].size(); i++) {
        //Action action = actionVectors[gcost][i];
        if (env.isRepeat(prevAct, actionVectors[gcost][i])) {
            continue;
        }
        int cost = 1;
        env.applyAction(actionVectors[gcost][i], root);
        bool p = DFS(threshold, gcost+cost, root, actionVectors[gcost][i], path);
        env.undoAction(actionVectors[gcost][i], root);
        if (p) {
            path->push_back(actionVectors[gcost][i]);
            return true;
        }
    }
    return false;
    
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
    Action initAct = Action(-10);
    std::vector<int> g = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    State goal = State(g);
    
    int index = 22;
    State benchmark = loadstpFile(index);
    //std::vector<int> s = {1,2,3,7,0,4,5,6,8,9,10,11,12,13,14,15};
    //State benchmark = State(s);
    
    Idastar_alg search = Idastar_alg(benchmark, goal);
    std::vector<Action> path = search.search(initAct);
    for (std::vector<Action>::size_type i=0; i < path.size(); i++) {
        cout << path[i].getAction() << ' ';
    }
    cout << endl;
    return 0;
}
