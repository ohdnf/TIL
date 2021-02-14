'''
mst + 인접행렬

7 11
0 5 60
0 1 32
0 2 31
0 6 51
1 2 21
2 4 46
2 6 25
3 4 34
3 5 18
4 5 40
4 6 51

결과
[0, 21, 31, 34, 46, 18, 25]
175
'''
V, E = map(int, input().split())
adj = [[0] * V for _ in range(V)]
for i in range(E):
    s, e, c = map(int, input().split())
    adj[s][e] = c
    adj[e][s] = c

# for row in adj:
#     print(row)

# key, pi, mst 준비
INF = float('inf')
key = [INF] * V
pi = [-1] * V
mst = [False] * V

# 시작점 선택: 0번
key[0] = 0
cnt = 0
result = 0
while cnt < V:
    # 아직 mst가 아니고, key가 최소인 정점(u) 선택
    min_key = INF
    u = -1
    for i in range(V):
        if not mst[i] and key[i] < min_key:
            min_key = key[i]
            u = i
    # u를 mst로 선택
    mst[u] = True
    result += min_key

    # key값을 갱신
    # u에 인접하고 아직 mst가 아닌 정점 w에서 key[w] > (u와 w 간 가중치) 면 갱신
    for w in range(V):
        if adj[u][w] > 0 and not mst[w] and key[w] > adj[u][w]:
            key[w] = adj[u][w]
            pi[w] = u

    cnt += 1

print(key)
print(pi)
print(result)