## *class* `collections.Counter`([*iterable-or-mapping*])

`Counter`는 hashable 객체의 셈을 도와주는 `dict` 서브클래스입니다. 요소를 dictionary의 key 값으로, 각 요소의 개수를 value 값으로 저장합니다. 개수는 0과 음수를 포함한 정수값입니다.  다른 언어의 bags이나 multisets과 유사한 클래스입니다.

_iterable_에서 요소의 개수를 세거나 _mapping_을 통해 초기화합니다.

```python
>>> from collections import Counter
>>> c = Counter()							# a new, empty counter
>>> c
Counter()
>>> c = Counter('gallahad')					# a new counter from an iterable
>>> c
Counter({'a': 3, 'l': 2, 'g': 1, 'h': 1, 'd': 1})
>>> c = Counter({'red': 4, 'blue': 2})		# a new counter from a mapping
>>> c
Counter({'red': 4, 'blue': 2})
>>> c = Counter(cats=4, dogs=8)				# a new counter from keyword args
>>> c
Counter({'dogs': 8, 'cats': 4})
```

Counter 객체는 dictionary interface를 따르지만 찾지 못하는 item에 대해 `KeyError` 대신 0을 반환합니다.

```python
>>> c = Counter(['eggs', 'ham'])
>>> c['bacon']                              # count of a missing element is zero
0
```

요소를 삭제하기 위해서 값을 0으로 설정하는 대신 `del`을 사용해야 합니다.

```python
>>> c['dogs'] = 0
>>> c
Counter({'cats': 4, 'dogs': 0})
>>> del c['dogs']
>>> c
Counter({'cats': 4})
```

_3.7 버전에서 바뀐 것:_ dict의 서브클래스로서, Counter는 삽입 순서를 기억하는 능력을 가지고 있습니다. Math operation을 Counter 객체에 적용할 때도 마찬가지입니다. 

### Counter의 메소드

#### `elements`()

요소를 개수만큼 반복한 iterator를 반환합니다. 개수가 0 이하인 경우는 무시됩니다.

```python
>>> c = Counter(a=4, b=2, c=0, d=-2)
>>> sorted(c.elements())
['a', 'a', 'a', 'a', 'b', 'b']
```

#### `most_common`([*n*])

_n_개의 요소들을 개수가 가장 많은 순으로 list에 담아 반환합니다. _n_이 생략되거나 `None`인 경우 모든 요소를 반환합니다.

```python
>>> Counter('abracadabra').most_common(3)
[('a', 5), ('b', 2), ('r', 2)]
```

#### `subtract`([*iterable-or-mapping*])

_iterable_이나 _mapping_ 또는 Counter 객체로 요소의 개수를 차감합니다. `dict.update()`와 비슷하지만 값이 대체되는 것이 아니라 개수가 감소한다는 차이가 있습니다.

```python
>>> c = Counter(a=4, b=2, c=0, d=-2)
>>> d = Counter(a=1, b=2, c=3, d=4)
>>> c.subtract(d)
>>> c
Counter({'a': 3, 'b': 0, 'c': -3, 'd': -6})
```

dictionary 메소드는 아래 두 개를 제외하고 동일하게 적용됩니다.

#### `fromkeys`(*iterable*)

이 메소드는 Counter 객체에 적용되지 않습니다.

#### `update`([*iterable-or-mapping*])

`subtract`에서 설명한 것과 마찬가지로, 값이 대체되는 것이 아니라 더해진다.



아래는 Counter 객체를 쓰는 일반적인 패턴입니다.

```python
sum(c.values())                 # total of all counts
c.clear()                       # reset all counts
list(c)                         # list unique elements
set(c)                          # convert to a set
dict(c)                         # convert to a regular dictionary
c.items()                       # convert to a list of (elem, cnt) pairs
Counter(dict(list_of_pairs))    # convert from a list of (elem, cnt) pairs
c.most_common()[:-n-1:-1]       # n least common elements
+c                              # remove zero and negative counts
```



> [Collection Counter Python Documentation](https://docs.python.org/3/library/collections.html#collections.Counter)



