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
#include <iterator>
#include <string.h>
#include <stdio.h>
#include <ctime>
using namespace std;


int num = 12;

unsigned long long factorial(int n) {
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

void lexicographicalRanking(std::vector<int> v) {
    int r = 0;
    int i, j, k;
    unsigned long long f;

    for (i = 0; i < num; i++) {
        k = v[i];
        for (j = 0; j < i; j++) {
            if (v[j] < v[i]) {
                k -= 1;
            }  
        }
        f = factorial(num-i-1);
	    r += k*f;
    }
}

int rank1(int n, std::vector<int> *state, std::vector<int> *dual) {
    if (n == 1) return 0;
    int s = (*state)[n-1];
    int l = (*dual)[n-1];
    (*state)[n-1] = (*state)[l];
    (*state)[l] = s;
    
    (*dual)[n-1] = (*dual)[s];
    (*dual)[s] = l;
    //unsigned long long f = factorial(n-1);
    return (s + n*rank1(n-1, state, dual));
}

void linearTimeRanking( std::vector<int> v, std::vector<int> *dual) {
    //int temp1=0, temp2=0;
    //int num = v.size();
    for (int i = 0; i < num; i++) {
    	//cout << v[i] << ' ';
        (*dual)[v[i]] = i;
    }
    rank1(num, &v, dual);
    //cout << "rank:" << rank1(nums, &v, dual) << endl;
    //return r;

}


void permutations() {
	std::vector<int> v(num), l(num);
	std::vector<int> dual(num);

	std::iota(v.begin(), v.end(), 0);
	std::iota(l.begin(), l.end(), 0);
	
	clock_t begin = clock();
	lexicographicalRanking(v);
	while (std::next_permutation(v.begin(),v.end())) {
		lexicographicalRanking(v);
	}
	clock_t end = clock();
	cout << " time elapsed: " << double(end - begin) / CLOCKS_PER_SEC << endl;
	
	begin = clock();
	linearTimeRanking(l, &dual);
	while (std::next_permutation(l.begin(),l.end())) {
		linearTimeRanking(l, &dual);
	}
	end = clock();
	cout << " time elapsed: " << double(end - begin) / CLOCKS_PER_SEC << endl;
}



int main(int argc, char const *argv[])
{
	permutations();
	return 0;
}