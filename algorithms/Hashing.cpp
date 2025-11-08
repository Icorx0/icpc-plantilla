#include <bits/stdc++.h>
using namespace std;

using i64 = long long;
using u64 = unsigned long long;
constexpr int MOD = 1e9 + 7;

struct StringHash {
    vector<u64> hashed, pwrs;
    u64 MOD_VAL;
    u64 BASE;

    StringHash(const string &s, u64 base = 257, u64 mod = MOD)
        : MOD_VAL(mod), BASE(base) {
        int n = s.length();
        hashed.resize(n + 1, 0);
        pwrs.resize(n + 1, 0);

        pwrs[0] = 1;
        for (int i = 1; i <= n; i++) {
            pwrs[i] = (pwrs[i - 1] * BASE) % MOD_VAL;
        }

        for (int i = 0; i < n; i++) {
            hashed[i + 1] = (hashed[i] * BASE + s[i]) % MOD_VAL;
        }
    }

    u64 getHash(int l, int r) {
        u64 hash = (hashed[r + 1] -(hashed[l] * pwrs[r - l + 1] % MOD_VAL) + MOD_VAL) % MOD_VAL;
        return hash;
    }
};

struct DoubleHash {
    StringHash hash1, hash2;

    DoubleHash(const string &s)
        : hash1(s, 257, 1e9 + 7), hash2(s, 353, 1e9 + 9) {}

    bool areEqual(int l1, int r1, int l2, int r2) {
        if (r1 - l1 != r2 - l2) return false;
        return hash1.getHash(l1, r1) == hash2.getHash(l2, r2) && hash1.getHash(l1, r1) == hash2.getHash(l2, r2);
    }
    
    bool areEqualStrict(int l1, int r1, int l2, int r2) {
        if (r1 - l1 != r2 - l2) return false;
        return hash1.getHash(l1, r1) == hash1.getHash(l2, r2) && hash2.getHash(l1, r1) == hash2.getHash(l2, r2);
    }
};