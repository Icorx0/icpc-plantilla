#include <bits/stdc++.h>
using namespace std;

vector<int> prefix_function(string s) {
    int n = (int)s.length();
    vector<int> pi(n);
    for (int i = 1; i < n; i++) {
        int j = pi[i-1];
        while (j > 0 && s[i] != s[j])
            j = pi[j-1];
        if (s[i] == s[j])
            j++;
        pi[i] = j;
    }
    return pi;
}

// Busqueda de patron P en texto T
vector<int> kmp_search(const string &P, const string &T) {
    string s = P + "#" + T;
    vector<int> pi = prefix_function(s);
    vector<int> matches;
    int m = P.length();
    for (int i = m + 1; i < s.length(); i++) {
        if (pi[i] == m) {
            matches.push_back(i - 2 * m); // posicion en T
        }
    }
    return matches;
}