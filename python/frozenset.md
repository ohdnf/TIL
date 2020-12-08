# frozenset

> set은 mutable하기 때문에 `add()`나 `remove()` 같은 메서드로 변경이 가능하다. 하지만 frozenset은 immutable하기 때문에 생성되고 나서 변경할 수 없다. 그렇기 때문에 hashable한 속성을 가지게 되어 dictionary key나 set 안에 요소로 사용이 가능하다.

```python
data = [[], [1, 2], [5], [1, 2, 5], [1, 2, 3, 4], [1, 2, 3, 6]]
data = set(frozenset(l) for l in data)
```

