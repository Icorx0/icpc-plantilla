#include <bits/stdc++.h>
#define ll long long
using namespace std;

struct FenwickTree {
    vector<int> bit;  // binary indexed tree
    int n;

    FenwickTree(int n) {
        this->n = n;
        bit.assign(n, 0);
    }

    FenwickTree(vector<int> const &a) : FenwickTree(a.size()) {
        for (size_t i = 0; i < a.size(); i++)
            add(i, a[i]);
    }

    ll sum(int r) {
        ll ret = 0;
        for (; r >= 0; r = (r & (r + 1)) - 1)
            ret += bit[r];
        return ret;
    }

    ll sum(int l, int r) {
        return sum(r) - (l == 0 ? 0LL : sum(l - 1));
    }

    ll add(int idx, int delta) {
        for (; idx < n; idx = idx | (idx + 1))
            bit[idx] += delta;
    }
};

// Gemini implementation
struct RangeFenwickTree {
    FenwickTree B1;
    FenwickTree B2;
    int n;

    RangeFenwickTree(int size) : n(size), B1(size), B2(size) {}

    ll prefix_sum(int idx) {
        return B1.sum(idx) * (idx + 1) - B2.sum(idx);
    }

    ll range_sum(int l, int r) {
        ll sum_r = prefix_sum(r);
        ll sum_l_minus_1 = (l == 0) ? 0LL : prefix_sum(l - 1);
        return sum_r - sum_l_minus_1;
    }

    void range_add(int l, int r, ll x) {
        B1.add(l, x);
        if (r + 1 < n) {
            B1.add(r + 1, -x);
        }

        B2.add(l, x * l);
        if (r + 1 < n) {
            B2.add(r + 1, -x * (r + 1));
        }
    }
};