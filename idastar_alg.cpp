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

using namespace std;

class Idastar_alg {
private:
	int expanded;
	int nextThreshold;
	State start;
	Heuristic* heu;
	Stp_env* env;
	std::vector<Action> path;	

public:
	Idastar_alg(State s, State g);
	~Idastar_alg();
	bool search(Action initAct);
	int DFS(int threshold, int gcost, State root, 
					Action prevAct, std::vector<Action> path);
	
};


Idastar_alg::Idastar_alg(State s, State g){
	expanded = 0;
	start = s;
	heu = new Heuristic(s, g);
	env = heu->getEnv(); 	
	nextThreshold = heu->HCost(s);
	path = {};
}

Idastar_alg::~Idastar_alg(){}

bool Idastar_alg::search(Action initAct){
	bool success = false;
	int threshold = nextThreshold;
	int gcost = 0;
	while (!success) {
		success = DFS(threshold, gcost, start, initAct, path);
		threshold = nextThreshold;
	}
	return success;

}

int Idastar_alg::DFS(int threshold, int gcost, State root, 
					Action prevAct, std::vector<Action> path){
	expanded += 1;
	int fcost = gcost + heu->HCost(root);
	if (env->isSuccess(root)) {
		return prevAct.getAction();
	}
	if (threshold < fcost) {
		if (nextThreshold > fcost || nextThreshold == threshold) {
			nextThreshold = fcost;
		}
		return -1;
	}

	std::vector<Action> actions = env->getActions(root);
	for (std::vector<Action>::size_type i = 0; i < actions.size(); i++) {
		Action action = actions[i];
		if (env->isRepeat(prevAct, action)) {
			continue;
		}
		int cost = 1; 
		env->applyAction(action, root);
		int p = DFS(threshold, gcost+cost, root, action, path);
		env->undoAction(action, root);
		if (p != -1) {
			path.push_back(action);
			return 1;
		}
	}
	return -1;

}

State loadstpFile(int index){

	std::vector<State> instances;
	std::string df = "./asg1/asg1/data/korf100.txt";
	std::fstream infile(df, std::ios_base::in);
	std::string line;
	while (std::getline(infile, line)) {
		std::istringstream buffer(line);
		std::vector<string> tokens{std::istream_iterator<string>(buffer),
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
	int index = 0;
	Action initAct = Action(-1);
	std::vector<int> g = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	State goal = State(g);
	State benchmark = loadstpFile(index);
	Idastar_alg search = Idastar_alg(benchmark, goal);
	search.search(initAct);

	return 0;
}