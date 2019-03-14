idastar: idastar_alg.cpp heuristic.hpp stp_env.hpp
		g++ -O3 -Wall -std=c++11 -o idastar idastar_alg.cpp
astar: astar_alg.cpp heuristic.hpp stp_env.hpp
		g++ -O3 -Wall -std=c++11 -o astar astar_alg.cpp
clean:
	rm -rf idastar astar