'''
다익스트라 + 인접리스트

6 11
0 1 3
0 2 5
1 2 2
1 3 6
2 1 1
2 3 4
2 4 6
3 4 2
3 5 3
4 0 3
4 5 6

결과

[0, 3, 5, 9, 11, 12]
'''

V, E = map(int, input().split())
adj = { node: list() for node in range(V) }
for _ in range(E):
    s, e, w = map(int, input().split())
    adj[s].append([e, w])

INF = float('inf')
dist = [INF] * V
selected = [False] * V

dist[0] = 0
cnt = 0
while cnt < V:
    min_dist = INF
    u = -1

    # 최소 거리 정점 찾기
    for node in range(V):
        if not selected[node] and dist[node] < min_dist:
            min_dist = dist[node]
            u = node
    selected[u] = True

    # 간선 완화
    for w, cost in adj[u]:  # 도착정점, 가중치
        if dist[w] > dist[u] + cost:
            dist[w] = dist[u] + cost
    
    cnt += 1

print(dist)