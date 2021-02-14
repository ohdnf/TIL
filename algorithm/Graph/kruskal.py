'''
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

7 11
0 1 2
0 2 2
0 5 8
1 2 1
1 3 19
2 5 5
3 4 7
3 5 11
3 6 15
4 5 9
4 6 3

결과

'''

def make_set(x):
    p[x] = x

def find_set(x):
    if p[x] == x: return x
    else:
        p[x] = find_set(p[x])
        return p[x]

def union(x, y):
    px = find_set(x)
    py = find_set(y)
    if rank[px] > rank[py]:
        p[py] = px
    else:
        p[px] = py
        if rank[px] == rank[py]:
            rank[py] += 1

V, E = map(int, input().split())
edges = [list(map(int, input().split())) for _ in range(E)]

# 간선을 간선가중치 기준으로 정렬
edges.sort(key=lambda x: x[2])
# print(edges)
p = [0] * V
rank = [0] * V
for i in range(V):
    make_set(i)

cnt = 0
result = 0
mst = list()
# 모든 간선에 대해서 반복 ==> V-1개의 간선이 선택될 때까지
for i in range(E):
    s, e, c = edges[i][0], edges[i][1], edges[i][2]
    # 사이클이면 스킵: 간선의 두 정점이 서로 같은 집합(find_set)이면 간선 선택
    if find_set(s) == find_set(e): continue
    # 간선 선택 => mst에 간선 정보 더하기 / 두 정점을 합친다(union)
    result += c
    mst.append(edges[i])
    union(s, e)
    cnt += 1
    if cnt == V-1: break
    
print(result)
print(mst)