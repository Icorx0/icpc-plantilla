#include <bits/stdc++.h>
using namespace std;

using i64 = long long;
using u64 = unsigned long long;
using i128 = __int128_t;
using u128 = __uint128_t;
using ld = long double;

constexpr int MOD = 1e9 + 7;

inline i64 addmod(i64 a, i64 b, i64 mod = MOD) {
  a %= mod;
  b %= mod;
  a += b;
  if (a >= mod)
    a -= mod;
  return a;
}

inline i64 submod(i64 a, i64 b, i64 mod = MOD) {
  a %= mod;
  b %= mod;
  a -= b;
  if (a < 0)
    a += mod;
  return a;
}

inline i64 mulmod(i64 a, i64 b, i64 mod = MOD) {
  return (i64)((i128)a * b % mod);
}

i64 binpow(i64 a, i64 e, i64 mod = MOD) {
  i64 r = 1 % mod;
  a %= mod;
  while (e) {
    if (e & 1)
      r = mulmod(r, a, mod);
    a = mulmod(a, a, mod);
    e >>= 1;
  }
  return r;
}

inline i64 lcm_ll(i64 a, i64 b) {
  if (a == 0 || b == 0)
    return 0;
  return a / std::gcd(a, b) * b;
}

// Devuelve el GCD de a y b, y x, y tales que ax + by = GCD(a, b)
i64 egcd(i64 a, i64 b, i64 &x, i64 &y) {
  if (!b) {
    x = 1;
    y = 0;
    return a;
  }
  i64 x1, y1;
  i64 g = egcd(b, a % b, x1, y1);
  x = y1;
  y = x1 - (a / b) * y1;
  return g;
}

// Inverso modular: usar Fermat si mod es primo, si no, usar egcd
inline i64 invmod(i64 a, i64 mod = MOD) {
  if (std::gcd(a, mod) != 1) {
    return -1; // no existe
  }
  // Fermat si MOD es primo
  // return binpow((a % mod + mod) % mod, mod - 2, mod);
  i64 x, y;
  egcd(a, mod, x, y);
  x %= mod;
  if (x < 0)
    x += mod;
  return x;
}

inline i64 moddiv(i64 a, i64 b, i64 mod = MOD) {
  i64 inv = invmod(b, mod);
  return inv == -1 ? -1 : mulmod((a % mod + mod) % mod, inv, mod);
}

// -------------------- Binomial coefficients --------------------
// Exact binomial coefficient (integer). For large n this may overflow i64.
inline i64 binom_exact(i64 n, i64 k) {
  if (k < 0 || k > n)
    return 0;
  k = min(k, n - k);
  i64 res = 1;
  // Compute iteratively: res = C(n, i) for i from 1..k
  for (i64 i = 1; i <= k; ++i) {
    // Multiply then divide; this stays integral at every step because
    // res = C(n, i-1) and res * (n - i + 1) / i = C(n, i).
    res = res * (n - i + 1) / i;
  }
  return res;
}

// Modular binomial coefficients using precomputed factorials.
// Usage: call init_fact(MAX_N) once (MAX_N >= maximum n you'll query).
static vector<i64> fact_mod, invfact_mod;

inline void init_fact(int N, i64 mod = MOD) {
  fact_mod.assign(N + 1, 1);
  invfact_mod.assign(N + 1, 1);
  for (int i = 1; i <= N; ++i)
    fact_mod[i] = mulmod(fact_mod[i - 1], i, mod);
  // invfact[N] = inverse of fact[N]
  invfact_mod[N] = invmod(fact_mod[N], mod);
  // compute invfact downwards
  for (int i = N; i > 0; --i)
    invfact_mod[i - 1] = mulmod(invfact_mod[i], i, mod);
}

inline i64 nCr_mod(int n, int r, i64 mod = MOD) {
  if (r < 0 || r > n)
    return 0;
  if ((int)fact_mod.size() <= n) {
    // Not initialized for this n; fallback to multiplicative method modulo (works if mod is prime)
    // This multiplicative method requires mod to be prime for modular division.
    i64 num = 1, den = 1;
    r = min(r, n - r);
    for (int i = 1; i <= r; ++i) {
      num = mulmod(num, n - r + i, mod);
      den = mulmod(den, i, mod);
    }
    i64 inv = invmod(den, mod);
    return mulmod(num, inv, mod);
  }
  return mulmod(fact_mod[n], mulmod(invfact_mod[r], invfact_mod[n - r], mod), mod);
}

// -------------------- Catalan numbers --------------------
// Exact Catalan number: C_n = binom(2n, n) / (n+1). May overflow for large n.
inline i64 catalan_exact(i64 n) {
  if (n < 0)
    return 0;
  return binom_exact(2 * n, n) / (n + 1);
}

// Catalan modulo MOD (MOD should be prime for invmod to work reliably)
inline i64 catalan_mod(int n, i64 mod = MOD) {
  if (n < 0)
    return 0;
  i64 c = nCr_mod(2 * n, n, mod);
  i64 inv = invmod(n + 1, mod);
  if (inv == -1) // no inverse
    return -1;
  return mulmod(c, inv, mod);
}