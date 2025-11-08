#include <bits/stdc++.h>
using i64 = long long;
using namespace std;
const i64 INF = 1e18;

void floydWarshall(vector<vector<i64>>& d, int N) {
    d.resize(N, vector<i64>(N, INF)); // all paths inf by default
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (d[i][k] < INF && d[k][j] < INF)
                    d[i][j] = min(d[i][j], d[i][k] + d[k][j]); 
            }
        }
    }
}