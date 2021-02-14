# 그래프



## 오일러 경로

> 모든 간선을 한 번씩 방문하는 유한 그래프

- 일명 한 붓 그리기가 가능한 그래프
- 모든 정점이 짝수 개의 차수(Degree)를 갖는다면 모든 다리를 한 번씩만 건너서 도달하는 것이 성립



## 해밀턴 경로

> 각 정점을 한 번씩 방문하는 무향 또는 유향 그래프 경로

- 최적 알고리즘이 없는 대표적인 **NP-완전** 문제
- 원래의 출발점으로 돌아오는 경로는 특별히 **해밀턴 순환**이라 지칭
  - 이중 최단 거리를 찾는 문제는 **외판원 문제**로 유명
- 다이나믹 프로그래밍 기법을 활용하면 최적화 가능

> #### 정리
>
> - 해밀턴 경로: 한 번만 방문하는 경로
> - 해밀턴 순환: 한 번만 방문하여 출발지로 돌아오는 경로
> - 외판원 문제: 한 번만 방문하여 출발지로 돌아오는 경로 중 가장 짧은 경로
>
> 따라서 `해밀턴 경로 ⊃ 해밀턴 순환 ⊃ 외판원 문제` 의 포함 관계가 성립



## 그래프 순회

> 그래프 탐색이라고도 불리우며 그래프의 각 정점을 방문하는 과정을 지칭



### DFS: Depth-First Search

> 깊이 우선 탐색. 주로 스택 또는 재귀로 구현

#### 재귀 구조로 구현

##### 수도코드

```
DFS(G, v)
	label v as discovered
	for all directed edges from v to w that are in G.adjacentEdges(v) do
		if vertex w is not labeled as discovered then
			recursively call DFS(G, w)
```

##### Python 코드

```python
def recursive_dfs(v, discovered=[]):
    discovered.append(v)
    for w in graph[v]:
        if not w in discovered:
            discovered = recursive_dfs(w, discovered)
    return discovered
```

> #### 백트래킹
>
> 백트래킹(Backtracking)은 해결책에 대한 후보를 구축해 나아가다 가능성이 없다고 판단되는 즉시 후보를 포기(Backtrack)해 정답을 찾아가는 범용적인 알고리즘으로 제약 충족 문제(Constraint Satisfaction Problems)에 특히 유용하다.
>
> #### 제약 충족 문제
>
> 수많은 제약 조건을 충족하는 상태를 찾아내는 수학 문제를 일컫는다.
>
> 합리적인 시간 내에 문제를 풀기 위해 휴리스틱과 조합 탐색 같은 개념을 함께 결합해 문제를 풀이한다. 대표적으로 스도쿠, 십자말 풀이, 8퀸 문제, 4색 문제 같은 퍼즐 문제와 배낭 문제, 문자열 파싱, 조합 최적화 문제 등이 있다.



#### 스택을 이용한 반복 구조로 구현

##### 수도코드

```
DFS-iterative(G, v)
	let S be a stack
	S.push(v)
	while S is not empty do
		v = S.pop()
		if v is not labeled as discovered then
			label v as discovered
			for all edges from v to w in G.adjacentEdges(v) do
				S.push(w)
```

##### Python 코드

```python
def iterative_dfs(start_v):
    discovered = []
    stack = [start_v]
    while stack:
        v = stack.pop()
        if v not in discovered:
            discovered.append(v)
            for w in graph[v]:
                stack.append(w)
    return discovered
```



### BFS: Breadth-First Search

> 너비 우선 탐색. 최단 경로를 찾는 다익스트라 알고리즘 등에 매우 유용하게 사용

#### 큐를 이용한 반복 구조로 구현

##### 수도코드

```
BFS(G, start_v)
	let Q be a queue
	label start_v as discovered
	Q.enqueue(start_v)
	while Q is not empty do
		v := Q.dequeue()
		if v is the goal then
			return v
		for all edges from v to w in G.adjacentEdges(v) do
			if w is not labeled as discovered then
				label w as discovered
				w.parent := v
				Q.enqueue(w)
```

##### Python 코드

```python
def iterative_bfs(start_v):
    discovered = [start_v]
    queue = [start_v]
    while queue:
        v = queue.pop(0)
        if v == goal:
            return discovered
        for w in graph[v]:
            if w not in discovered:
                discovered.append(w)
                queue.append(w)
    return discovered
```

#### 재귀 구현 불가

BFS는 재귀로 동작하지 않는다. 큐를 이용하는 반복 구현만 가능하다.

