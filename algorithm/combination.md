# 조합(Combination)

## [Python Documentation](https://docs.python.org/3/library/itertools.html#itertools.combinations)

`itertools` 모듈 활용

```python
from itertools import combinations

for c in combinations('ABCD', 2):
    print(c)

"""
('A', 'B')
('A', 'C')
('A', 'D')
('B', 'C')
('B', 'D')
('C', 'D')
"""
```



직접 구현

```python
def combinations(iterable, r):
    # combinations('ABCD', 2) --> AB AC AD BC BD CD
    # combinations(range(4), 3) --> 012 013 023 123
    pool = tuple(iterable)
    n = len(pool)
    if r > n:
        return
    indices = list(range(r))
    yield tuple(pool[i] for i in indices)
    while True:
        for i in reversed(range(r)):
            if indices[i] != i + n - r:
                break
        else:
            return
        indices[i] += 1
        for j in range(i+1, r):
            indices[j] = indices[j-1] + 1
        yield tuple(pool[i] for i in indices)
```



## DFS 활용

```python
def dfs(level, start):
    global cnt
    if level == m:
        print(*res)
        cnt += 1
    else:
        for num in range(start, n+1):
            if not chk[num]:
                chk[num] = True
                res[level] = num
                dfs(level+1, num+1)
                chk[num] = False


if __name__ == '__main__':
    n, m = map(int, input().split())
    chk = [False] * (n+1)
    res = [0] * m
    cnt = 0
    dfs(0, 1)
    print(cnt)

```