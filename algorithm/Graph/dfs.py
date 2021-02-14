import sys
sys.stdin = open('input.txt')

n, e = map(int, input().split())

matrix = [[] for _ in range(n+1)]
visited = [False for _ in range(n+1)]

for _ in range(e):
    f, t = map(int, input().split())
    matrix[f].append(t)
    matrix[t].append(f)

visit_order = []
stack = []
v = 1
def dfs(v):
    visited[v] = True
    visit_order.append(v)
    # if not all([visited[w] for w in matrix[v]]):
    stack.append(v)
    while stack:
        print(f'v: {v}, matrix[v]: {matrix[v]}')
        print(f'visited: {visited}')
        for w in matrix[v]:
            if not visited[w]:
                visited[w] = True
                visit_order.append(w)
                stack.append(w)
                v = w
                break
        else:
            v = stack.pop()
        print(f'stack: {stack}')
dfs(v)
print(visit_order)