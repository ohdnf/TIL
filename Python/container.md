# Slicing

## Container 객체의 [](대괄호) slicing은 __getitem__ 메소드를 사용하는 것

```
>>> range(10)[slice(4, 7, 2)]
range(4, 7, 2)
>>> range(10).__getitem__(slice(4, 7, 2))
range(4, 7, 2)
```

## Slice에 요소 할당

```
>>> a = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
>>> a[2:5] = ['a', 'b', 'c']
>>> a
[0, 1, 'a', 'b', 'c', 5, 6, 7, 8, 9]
```

할당할 범위가 일치하지 않는 경우 리스트 요소 개수도 줄어든다.

```
>>> a = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
>>> a[2:5] = ['a']
>>> a
[0, 1, 'a', 5, 6, 7, 8, 9]
```


# OrderedDict
Dictionary의 Key 순서를 보장하려면 collection 모듈의 OrderedDict를 사용(파이썬 3.5 이하)

```
>>> from collections import OrderedDict
>>> lux = OrderedDict({'health': 490, 'health': 800, 'mana': 334, 'melee': 550, 'armor': 18.72})
>>> lux
OrderedDict([('health', 800), ('mana', 334), ('melee', 550), ('armor', 18.72)])
```
