import sys
sys.stdin = open('input.txt')

def dfs(v):
    visited[v] = 1
    print(v, end=' ')

    for w in g[v]:
        if not visited[w]:
            dfs(w)


v, e = map(int, input().split())
g = [[] for _ in range(v+1)]
visited = [0 for _ in range(v+1)]

for i in range(e):
    # introduction to algorithms
    u, v = map(int, input().split())
    g[u].append(v)
    g[v].append(u)

