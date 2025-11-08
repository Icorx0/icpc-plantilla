#include <bits/stdc++.h>
using namespace std;

using i64 = long long;
using u64 = unsigned long long;
using i128 = __int128_t;
using u128 = __uint128_t;
using ld = long double;

constexpr int INF32 = 0x3f3f3f3f; // ~1e9
constexpr i64 INF64 = (i64)4e18;
constexpr ld EPS = 1e-12L;
constexpr ld PI = 3.14159265358979323846264338327950288L;
constexpr int MOD = 1e9 + 7;

#define fastio                                                                 \
  ios::sync_with_stdio(false);                                                 \
  cin.tie(nullptr);                                                            \
  cout.tie(nullptr);

// ---------- optional ----------
template <class T> using V = vector<T>;
template <class K, class Val> using umap = unordered_map<K, Val>;
template <class K> using uset = unordered_set<K>;

#define rep(i, n) for (int i = 0; i < n; ++i)
#define per(i, n) for (int i = n - 1; i >= 0; --i)
#define reps(i, a, b) for (int i = a; i < b; ++i)
#define pers(i, a, b) for (int i = b - 1; i >= a; --i)
#define all(x) begin(x), end(x)
#define rall(x) rbegin(x), rend(x)
#define len(x) (x.size())

template <class T> using MinHeap = priority_queue<T, vector<T>, greater<T>>;
template <class T> using MaxHeap = priority_queue<T>;
// ---------- optional ----------

void solve() {}

int main() {
  fastio;
  int T = 1;
  // cin >> T;
  while (T--) solve();
  return 0;
}