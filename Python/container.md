# Slicing

## Container ��ü�� [](���ȣ) slicing�� __getitem__ �޼ҵ带 ����ϴ� ��

```
>>> range(10)[slice(4, 7, 2)]
range(4, 7, 2)
>>> range(10).__getitem__(slice(4, 7, 2))
range(4, 7, 2)
```

## Slice�� ��� �Ҵ�

```
>>> a = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
>>> a[2:5] = ['a', 'b', 'c']
>>> a
[0, 1, 'a', 'b', 'c', 5, 6, 7, 8, 9]
```

�Ҵ��� ������ ��ġ���� �ʴ� ��� ����Ʈ ��� ������ �پ���.

```
>>> a = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
>>> a[2:5] = ['a']
>>> a
[0, 1, 'a', 5, 6, 7, 8, 9]
```


# OrderedDict
Dictionary�� Key ������ �����Ϸ��� collection ����� OrderedDict�� ���(���̽� 3.5 ����)

```
>>> from collections import OrderedDict
>>> lux = OrderedDict({'health': 490, 'health': 800, 'mana': 334, 'melee': 550, 'armor': 18.72})
>>> lux
OrderedDict([('health', 800), ('mana', 334), ('melee', 550), ('armor', 18.72)])
```
