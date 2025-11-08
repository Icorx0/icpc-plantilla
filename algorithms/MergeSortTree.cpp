#include <bits/stdc++.h>
using namespace std;

struct MergeSortTree {
    vector<vector<int>> heap;
    // s.build(array, 1, 0, n-1);
    void build(vector<int> &array, int v, int tl, int tr) {
        if(heap.size() <= v) heap.resize(v+1);
        if(tl == tr) heap[v] = vector<int> (1, array[tl]);
        else {
            int tm = (tl + tr) / 2;
            build(array, 2*v, tl, tm);
            build(array, 2*v+1, tm+1, tr);
            merge(heap[2*v].begin(), heap[2*v].end(), heap[2*v+1].begin(),
                heap[2*v+1].end(), back_inserter(heap[v]));
        }
    }
    // s.get(1, 0, n-1, l, r);
    vector<int> get(int v, int tl, int tr, int l, int r) {
        if(l > r) return vector<int> ();
        if(l == tl and r == tr) return heap[v];
        int tm = (tl + tr) / 2;
        vector<int> v1 = get(v*2, tl, tm, l, min(r, tm)), v2 = get(v*2+1, tm+1, tr, max(l, tm+1), r), result;
        merge(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(result));
        return result;
    }
    // s.update(1, 0, n-1, pos, x);
    void update(int v, int tl, int tr, int pos, int x) {
        if(tl == tr) {
            heap[v] = vector<int> (1, x);
        } else {
            int tm = (tl + tr) / 2;
            if(pos <= tm) update(2*v, tl, tm, pos, x);
            else update(2*v+1, tm+1, tr, pos, x);
            heap[v] = vector<int> ();
            merge(heap[2*v].begin(), heap[2*v].end(), heap[2*v+1].begin(),
                heap[2*v+1].end(), back_inserter(heap[v]));
        }
    }
};