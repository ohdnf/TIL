import sys
sys.stdin = open('input.txt')

def dfs(v):
    stack = []
    stack.append(v)
    visited[v] = 1
    print(v, end='-')

    while stack:
        p = v
        for w in g[v]:
            if not visited[w]:
                stack.append(w)
                v = w
                visited[w] = 1
                print(w, end='-')
                break
        else:
            if p == v:
                v = stack.pop()


v, e = map(int, input().split())
g = [[] for _ in range(v+1)]
visited = [0 for _ in range(v+1)]

for i in range(e):
    # introduction to algorithms에서는 node를 u, v로 표현
    u, v = map(int, input().split())
    g[u].append(v)
    g[v].append(u)

