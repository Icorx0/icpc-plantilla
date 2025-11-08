#include <bits/stdc++.h>
using namespace std;

const int INF = 1000000000;
vector<vector<pair<int, int>>> adj;
int n_vertices;

void dijkstra(int source, vector<int> &distances, int current_time) {
    distances.assign(n_vertices, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> tour;

    distances[source] = current_time;
    tour.push({distances[source], source});
    
    while(!tour.empty()) {
        auto u = tour.top();
        tour.pop();

        if (distances[u.second] != u.first) continue;
        for (pair<int, int> &v: adj[u.second]) {
            int d = u.first + v.second;
            if (d < distances[v.first]) {
                distances[v.first] = d;
                tour.push({d, v.first});
            }
        }
    }
}
