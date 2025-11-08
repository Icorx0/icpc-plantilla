#include <bits/stdc++.h>
using namespace std;

struct SparseTable {
    int N, K;
    vector<vector<int>> table;
    vector<int> logs;

    // Constructor to build the table
    SparseTable(const vector<int>& arr) {
        N = arr.size();
        K = floor(log2(N)) + 1; // Max power of 2
        table.assign(N, vector<int>(K));
        logs.resize(N + 1);

        logs[1] = 0;
        for (int i = 2; i <= N; i++) {
            logs[i] = logs[i / 2] + 1;
        }

        for (int i = 0; i < N; i++) {
            table[i][0] = arr[i];
        }

        for (int j = 1; j < K; j++) {
            for (int i = 0; i + (1 << j) <= N; i++) {
                table[i][j] = min(table[i][j - 1], 
                                  table[i + (1 << (j - 1))][j - 1]);
            }
        }
    }

    // Query the range [L, R] (inclusive) in O(1)
    int query(int L, int R) {
        // Find largest k such that 2^k <= (R - L + 1)
        int k = logs[R - L + 1];
        
        // Return min of the two overlapping ranges
        return min(table[L][k], table[R - (1 << k) + 1][k]);
    }
};