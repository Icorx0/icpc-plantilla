#include <bits/stdc++.h>
using namespace std;
// A centroid of a tree is defined as a node such that when the tree is rooted at it, no other nodes have a subtree of size greater than  $\frac{N}{2}$

const int maxn = 200010;

int n;
vector<int> adj[maxn];
int subtree_size[maxn];

// must be called first at 0 to fill subtree_size
int get_subtree_size(int node, int parent = -1) {
	int &res = subtree_size[node];
	res = 1;
	for (int i : adj[node]) {
		if (i == parent) { continue; }
		res += get_subtree_size(i, node);
	}
	return res;
}

int get_centroid(int node, int parent = -1) {
	for (int i : adj[node]) {
		if (i == parent) { continue; }

		if (subtree_size[i] * 2 > n) { return get_centroid(i, node); }
	}
	return node;
}