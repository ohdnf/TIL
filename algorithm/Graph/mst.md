# Minimum Spanning Tree

## Prim

## Kruskal

- Cycle을 어떻게 검증?
  - 연결할 정점 간 대표자가 같을 때 Cycle 발생

## Dijkstra

- 시작 정점에서 거리가 최소인 정점을 선택해 나가면서 최단 경로를 구하는 방식

- 시작정점(s)에서 끝정점(t)까지의 최단 경로에 정점 x가 존재
- 이때, 최단경로는 s에서 x까지의 최단 경로와 x에서 t까지의 최단경로로 구성

- 탐욕 기법을 사용한 알고리즘으로 MST의 Prim 알고리즘과 유사

- `d[node]`: 시작점부터 `node`까지의 (현재까지 알아낸 최단)거리

  ```
  # Edge Relaxation(간선완화)
  if d[v] > d[u] + cost(u.v)
  d[v] <- d[u] + cost(u.v)
  ```

