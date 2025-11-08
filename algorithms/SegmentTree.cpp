#include <bits/stdc++.h>
using namespace std;

struct SegmentTree {
    vector<int> heap;
    vector<long long> lazy;

    // s.init(n);
    void init(int n) {
        int size = 4 * n + 1;
        heap.assign(size, 0);
        lazy.assign(size, 0);
    }

    // s.build(array, 1, 0, n-1);
    void build(vector<int> &array, int v, int tl, int tr) {
        if(tl == tr) heap[v] = array[tl];
        else {
            int tm = (tl + tr) / 2;
            build(array, 2*v, tl, tm);
            build(array, 2*v+1, tm+1, tr);
            heap[v] = heap[2*v] + heap[2*v+1];
        }
    }

    // s.sum(1, 0, n-1, l, r);
    int sum(int v, int tl, int tr, int l, int r) {
        if(l > r) return 0;
        if(l == tl and r == tr) return heap[v];
        int tm = (tl + tr) / 2;
        return sum(v*2, tl, tm, l, min(r, tm)) + sum(v*2+1, tm+1, tr, max(l, tm+1), r);
    }

    // s.update(1, 0, n-1, pos, x); in position pos set value x
    void update(int v, int tl, int tr, int pos, int x) {
        if(tl == tr) heap[v] = x;
        else {
            int tm = (tl + tr) / 2;
            if(pos <= tm) update(2*v, tl, tm, pos, x);
            else update(2*v+1, tm+1, tr, pos, x);
            heap[v] = heap[2*v] + heap[2*v+1];
        }
    }

    // push for lazy propagation
    void push(int v, int tl, int tr) {
        if (lazy[v] != 0 && tl != tr) {
            int tm = (tl + tr) / 2;
            long long addval = lazy[v];
            heap[2*v] += addval * (tm - tl + 1);
            heap[2*v+1] += addval * (tr - tm);
            lazy[2*v] += addval;
            lazy[2*v+1] += addval;
        }
        lazy[v] = 0;
    }

    // s.lazyupdate(1, 0, n-1, l, r, addend); // add 'addval' to range [l, r]
    void lazyupdate(int v, int tl, int tr, int l, int r, int addval) {
        push(v, tl, tr); // Propagate any pending updates
        if (l > r) return;
        if (l > tr || r < tl) return;
        if (l <= tl && tr <= r) {
            // Apply update to the current heap value
            heap[v] += (long long)addval * (tr - tl + 1);
            // Mark the lazy tag for future children updates
            lazy[v] += addval;             
            return;
        }

        int tm = (tl + tr) / 2;
        lazyupdate(2*v, tl, tm, l, r, addval);
        lazyupdate(2*v+1, tm+1, tr, l, r, addval);
        heap[v] = heap[2*v] + heap[2*v+1];
    }
};
