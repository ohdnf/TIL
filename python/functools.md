## `functools.reduce`(*function*, *iterable*[, *initializer*])

두 arguments에 *function*을 왼쪽에서 오른쪽으로 누적적으로 적용해 *iterable*을 단일 값으로 만듭니다. 예를 들어, `reduce(lambda x, y: x+y, [1, 2, 3, 4, 5])`는 `((((1+2)+3)+4)+5)`로 계산됩니다. *x*는 누적 값이고, *y*는 *iterable*에서 오는 새로운 값입니다. *initializer*가 존재하면 *iterable*의 item을 계산하기 전에 초기값으로 설정됩니다. *initializer*가 없고 *iterable*도 한 개의 값을 가지면 첫 번째 값이 반환됩니다.



[Programmers 코딩테스트 연습 > 해시 > 위장](https://programmers.co.kr/learn/courses/30/lessons/42578?language=python3)

```python
from collections import Counter
from functools import reduce


def solution(clothes):
    return reduce(lambda x, y: x*(y+1), [v for v in Counter([category for name, category in clothes]).values()], 1) - 1


if __name__ == '__main__':
    clothes = [["yellow_hat", "headgear"], ["blue_sunglasses", "eyewear"], ["green_turban", "headgear"]]
    print(solution(clothes))
```

