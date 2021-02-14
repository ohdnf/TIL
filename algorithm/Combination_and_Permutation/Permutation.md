# 순열(Permutation)

## [Python Documentation](https://docs.python.org/3/library/itertools.html#itertools.permutations)

`itertools` 모듈 활용

```python
from itertools import permutations

for p in permutations('ABCD', 2):
    print(p)

"""
('A', 'B')
('A', 'C')
('A', 'D')
('B', 'A')
('B', 'C')
('B', 'D')
('C', 'A')
('C', 'B')
('C', 'D')
('D', 'A')
('D', 'B')
('D', 'C')
"""
```



모듈과 유사한 코드

```python
def permutations(iterable, r=None):
    # permutations('ABCD', 2) --> AB AC AD BA BC BD CA CB CD DA DB DC
    # permutations(range(3)) --> 012 021 102 120 201 210
    pool = tuple(iterable)
    n = len(pool)
    r = n if r is None else r
    if r > n:
        return
    indices = list(range(n))
    cycles = list(range(n, n-r, -1))
    yield tuple(pool[i] for i in indices[:r])
    while n:
        for i in reversed(range(r)):
            cycles[i] -= 1
            if cycles[i] == 0:
                indices[i:] = indices[i+1:] + indices[i:i+1]
                cycles[i] = n-i
            else:
                j = cycles[i]
                indices[i], indices[-j] = indices[-j], indices[i]
                yield tuple(pool[i] for i in indices[:r])
                break
        else:
            return
```



## DFS 활용

```python
def dfs(depth, nums):
    global res
    if depth == m:
        res += 1
        print(' '.join(list(nums)))
    else:
        for num in range(1, n+1):
            if not chk[num]:
                chk[num] = True
                dfs(depth+1, nums+str(num))
                chk[num] = False


if __name__ == '__main__':
    n, m = map(int, input().split())
    chk = [False] * (n+1)
    res = 0
    dfs(0, '')
    print(res)

```