#include <bits/stdc++.h>
using namespace std;

bool is_set(unsigned int number, int x) {
    return (number >> x) & 1;
}

int set_bit(int number, int x) {
    return number | (1 << x);
}

int clear_bit(int number, int x) {
    return number & ~(1 << x);
}

int toggle_bit(int number, int x) {
    return number ^ (1 << x);
}

int modify_bit(int number, int x, bool val) {
    return (number & ~(1 << x)) | (val << x);
}

bool isDivisibleByPowerOf2(int n, int k) {
    int powerOf2 = 1 << k;
    return (n & (powerOf2 - 1)) == 0;
}

bool isPowerOfTwo(unsigned int n) {
    return n && !(n & (n - 1));
}

int nextPowerOfTwo(int n) {
    n--;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    return n + 1;
}

// Contar bits (g++ >= 20)
long long countSetBits(unsigned int n) {
    long long count = 0;
    while (n > 0) {
        int x = std::bit_width(n) - 1;
        if (x > 0) {
            count += (long long)x * (1LL << (x - 1));
        }
        n -= (1U << x);
        count += n + 1;
    }
    return count;
}

// Contar bits (g++ < 20)
long long countSetBits(unsigned int n) {
    long long count = 0;
    while (n > 0) {
        int x = static_cast<int>(std::log2(n));
        if (x > 0) {
            count += (long long)x * (1LL << (x - 1));
        }
        n -= (1U << x);
        count += n + 1;
    }
    return count;
}

int countSetBitsBuiltin(int n) {
    return __builtin_popcount(n);
}

int countSetBitsLong(long long n) {
    return __builtin_popcountll(n);
}

// Posición de bits
int lowestSetBit(int n) {
    return __builtin_ffs(n) - 1; // First set bit (0-indexed)
}

int highestSetBit(int n) {
    return 31 - __builtin_clz(n); // Count leading zeros
}

int trailingZeros(int n) {
    return __builtin_ctz(n);
}

int leadingZeros(int n) {
    return __builtin_clz(n);
}

// Operaciones con máscaras de bits
int getAllSetBits(int n) {
    return (1 << n) - 1; // Máscara con los primeros n bits en 1
}

int getRangeMask(int l, int r) {
    // Máscara con bits de l a r en 1 (0-indexed)
    return ((1 << (r - l + 1)) - 1) << l;
}

int clearRightmostSetBit(int n) {
    return n & (n - 1);
}

int isolateRightmostSetBit(int n) {
    return n & (-n);
}

int isolateRightmostZeroBit(int n) {
    return ~n & (n + 1);
}

int setRightmostZeroBit(int n) {
    return n | (n + 1);
}

// Iterar sobre subconjuntos
void iterateSubsets(int mask) {
    // Iterar sobre todos los subconjuntos de mask
    for (int s = mask; s > 0; s = (s - 1) & mask) {
        // Procesar subconjunto s
    }
}

void iterateAllMasks(int n) {
    // Iterar sobre todas las máscaras de n bits
    for (int mask = 0; mask < (1 << n); mask++) {
        // Procesar mask
    }
}

void iterateSetBits(int mask) {
    // Iterar sobre las posiciones de los bits en 1
    while (mask) {
        int pos = __builtin_ctz(mask);
        // Procesar posición pos
        mask &= mask - 1; // Eliminar el bit más bajo
    }
}

// Operaciones avanzadas
int reverseBits(unsigned int n) {
    n = ((n >> 1) & 0x55555555) | ((n & 0x55555555) << 1);
    n = ((n >> 2) & 0x33333333) | ((n & 0x33333333) << 2);
    n = ((n >> 4) & 0x0F0F0F0F) | ((n & 0x0F0F0F0F) << 4);
    n = ((n >> 8) & 0x00FF00FF) | ((n & 0x00FF00FF) << 8);
    n = (n >> 16) | (n << 16);
    return n;
}

int swapBits(int n, int i, int j) {
    // Intercambiar bits en posiciones i y j
    if (((n >> i) & 1) != ((n >> j) & 1)) {
        n ^= (1 << i) | (1 << j);
    }
    return n;
}

bool parityBit(unsigned int n) {
    // true si número impar de bits en 1
    return __builtin_parity(n);
}

// Operaciones con máscaras de bits para DP y combinatoria
int addElement(int mask, int pos) {
    return mask | (1 << pos);
}

int removeElement(int mask, int pos) {
    return mask & ~(1 << pos);
}

bool hasElement(int mask, int pos) {
    return (mask >> pos) & 1;
}

int maskSize(int mask) {
    return __builtin_popcount(mask);
}

int complementMask(int mask, int n) {
    // Complemento de mask con respecto a n bits
    return ((1 << n) - 1) ^ mask;
}

int unionMask(int a, int b) {
    return a | b;
}

int intersectionMask(int a, int b) {
    return a & b;
}

int differenceMask(int a, int b) {
    return a & ~b;
}

bool isSubset(int subset, int mask) {
    return (subset & mask) == subset;
}

// XOR útil
int xorRange(int l, int r) {
    // XOR de todos los números de l a r
    auto xor_till = [](int n) {
        int mod = n % 4;
        if (mod == 0) return n;
        if (mod == 1) return 1;
        if (mod == 2) return n + 1;
        return 0;
    };
    return xor_till(r) ^ xor_till(l - 1);
}

int findXorPair(int arr[], int n) {
    // XOR de todos los elementos
    int xor_all = 0;
    for (int i = 0; i < n; i++) {
        xor_all ^= arr[i];
    }
    return xor_all;
}

// Máximo XOR de dos números en un rango
int maxXOR(int l, int r) {
    int xor_val = l ^ r;
    int msb = highestSetBit(xor_val);
    return (1 << (msb + 1)) - 1;
}
