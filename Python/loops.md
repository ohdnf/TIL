# Looping

### while문

```
def solution(priorities, location):
    queue =  [(i,p) for i,p in enumerate(priorities)]
    answer = 0
    while True:
        cur = queue.pop(0)
        if any(cur[1] < q[1] for q in queue):
            queue.append(cur)
        else:
            answer += 1
            if cur[0] == location:
                return answer
```

* `return`으로 while문을 종료하고 나올 수 있다.
    * method 안에서 쓰일 때만 `return` 사용
* `any`로 리스트 탐색
