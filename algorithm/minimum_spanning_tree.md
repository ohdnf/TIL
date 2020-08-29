## 신장 트리(Spanning Tree)

N개의 정점을 포함하는 무향 그래프에서 N개의 정점과 N-1개의 간선으로 구성된 트리



### 최소 신장 트리(Minimum Spanning Tree)

가중치 그래프에서 신장 트리를 구성하는 간선들의 가중치의 합이 최소인 신장트리



### 프림 알고리즘(Prim's algorithm)

한 정점에 연결된 간선들 중 하나씩 선택하면서 최소 신장 트리를 만들어가는 방식



1. 임의의 정점을 하나 선택해서 시작
2. 선택한 정점들과 인접하는 정점들 중에 최소 비용의 간선이 존재하는 정점 선택
3. 모든 정점이 선택될 때까지 두 번째 과정 반복



#### 프림 알고리즘의 동작 과정

두 종류의 **상호 배타 집합들(2 disjoint-sets)** 정보 필요

1. 트리 정점들(Tree vertices)

   최소 신장 트리를 만들기 위해 선택된 정점들

2. 비트리 정점들(Non-tree vertices)

   선택되지 않은 정점들



```python
def MST_PRIM(G, s):		# G: 그래프, s: 시작 정점
    weight = [float('inf')] * N
    pi = [None] * N
    visited = [False] * N
    weight[s] = 0
    
    for _ in range(N):
        minIndex = -1
        minWeight = float('inf')
        for i in range(N):
            if not visited[i] and weight[i] < minWeight:
                minWeight = weight[i]
                minIndex = i
        visited[minIndex] = True
        for v, val in G[minIndex]:
            if not visited[v] and val < weight[v]:
                weight[v] = val
                pi[v] = minIndex
```



### 크루스칼(Kruskal) 알고리즘

사이클이 발생하지 않도록 최소 가중치 간선을 하나씩 선택해서 최소 신장 트리를 찾는 알고리즘

> 프림 알고리즘은 하나의 트리를 확장시켜나가는 방식이지만, 크루스칼 알고리즘은 간선을 선택해 나가는 과정에 여러 개의 트리들이 존재함

- 초기 상태는 n개의 정점들이 각각 하나의 트리
  - 하나의 정점을 포함하는 n개의 상호 배타 집합
- 간선을 선택하면 간선의 두 정점이 속한 트리가 연결되고 하나의 집합으로 합쳐짐
- 선택한 간선의 두 정점이 이미 연결된 트리에 속한 정점들일 경우 사이클이 생김
  - 두 정점에 대해 같은 집합의 원소 여부 검사로 확인



#### 크루스칼 알고리즘의 동작 과정

1. 최초, 모든 간선을 가중치에 따라 **오름차순**으로 정렬
2. **가중치가 가장 낮은 간선부터 선택**하면서 트리 증가시킴
   - 사이클이 존재하면 다음으로 가중치가 낮은 간선 선택
3. n-1개의 간선이 선택될 때까지 두 번째 과정을 반복



```python
def MST_KRUSKAL(G):
    mst = []
    
    for i in range(N):
        make_set(i)
        
    G.sort(key=lambda t: t[2])
    
    mst_cost = 0
    
    while len(mst) < N-1:
        u, v, val = G.pop(0)
        if find_set(u) != find_set(v):
            union(u, v)
            mst.append((u, v))
            mst_cost += val
```

- 간선 선택 과정에서 생성되는 트리를 관리하기 위해 **상호 배타 집합 사용**
  - 트리에 속한 노드들은 자신을 루트로 하는 **서브트리의 높이를 랭크(Rank)라는 이름으로 저장**
- 선택한 간선으로 두 개의 트리가 한 개의 트리로 합쳐질 때 **각 트리에 해당하는 상호 배타 집합을 Union연산으로 합침**
  - 랭크 값이 작은 트리를 랭크 값이 큰 트리의 서브트리로 포함시킬 경우 **트리에 포함된 노드들의 랭크 값 수정 불필요**