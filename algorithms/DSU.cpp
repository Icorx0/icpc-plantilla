#include <bits/stdc++.h>
using namespace std;

struct DSU {
    vector<int> parent, size;
    DSU(int n) {
        parent.resize(n);
        size.resize(n);
    }
    void make_set(int v) {
        parent[v] = v;
        size[v] = 1;
    }
    int find_set(int v) {
        if(v == parent[v]) return v;
        return parent[v] = find_set(parent[v]);
    }
    void union_sets(int a, int b) {
        a = find_set(a);
        b = find_set(b);
        if(a == b) return;
        if(size[a] < size[b]) swap(a, b);
        parent[b] = a;
        size[a] += size[b];
    }
};