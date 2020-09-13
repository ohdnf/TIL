# Python의 `yield` 이해하기

> Stack Overflow의 [What does the “yield” keyword do?](https://stackoverflow.com/questions/231767/what-does-the-yield-keyword-do) 글을 번역하며 이해하기

먼저 `yield`가 무엇인지 알기 위해선 generators가 무엇인지 알아야 한다. generators를 이해하기 전에 iterables를 이해해야 한다.

## Iterables

배열과 같이 요소를 하나씩 읽는 것을 iteration이라고 한다.

```python
>>> mylist = [1, 2, 3]
>>> for i in mylist:
...     print(i)
1
2
3
```

`mylist`는 iterable이다. *list comprehension* 또한 배열이므로 iterable이다.

```python
>>> mylist = [x*x for x in range(3)]
>>> for i in mylist:
...    print(i)
0
1
4
```

즉, `for ... in ...`을 사용할 수 있는 모든 Python 객체(`list`, `string`, 파일 등...)는 iterable이다.
<br>
이러한 iterable은 원하는 만큼 값을 **메모리에 저장**하고 읽을 수 있다. 하지만 *값이 많을 경우*, 메모리에 모두 저장하는 것을 피하고 싶을 때가 있다.

## Generators

Generator는 **단 한 번만 iterate**할 수 있는 iterator다. Generator는 모든 값을 메모리에 저장하는 대신, 순차적으로 값들을 그때 그때 계산하고 저장하지 않는다.

```python
>>> mygenerator = (x*x for x in range(3))
>>> for i in mygenerator:
...     print(i)
0
1
4
>>> for i in mygen:
... 	print(i)
	
>>> 
```

`[]`가 `()`로 바뀐 것뿐이지만, generator는 한 번만 실행되기 때문에 두 번 `for i in mygenerator`를 실행하면 아무 것도 얻을 수 없다. generator는 0을 계산한 뒤 값을 지우고, 1을 계산한 뒤 값을 지우고, 4를 계산한 뒤 값을 지운다.

## Yield

`yield`는 `return`과 비슷하게 사용되지만, generator를 반환한다는 점에서 다르다.

```python
>>> def createGenerator():
...    mylist = range(3)
...    for i in mylist:
...        yield i*i
...
>>> mygenerator = createGenerator() # generator 생성
>>> print(mygenerator) # mygenerator는 객체다!
<generator object createGenerator at 0xb7555c34>
>>> for i in mygenerator:
...     print(i)
0
1
4
>>> for i in mygenerator:
...     print(i)

>>>
```

예시를 보면, `createGenerator` 함수가 딱 한 번 사용할 generator 객체를 반환한다는 것을 알 수 있다.

`yield`를 마스터하기 위해선, **함수를 호출할 때, 함수 몸통(function body)이 실행되지 않는다**는 것을 이해해야 한다. 함수는 generator 객체를 반환할 뿐이다.

함수 몸통은 `for` 구문이 generator를 사용할 때마다 마지막으로 중단되었던 곳(`yield`)으로 돌아가 다시 실행된다.

어려운 부분:

처음 `for` 구문이 함수 안에서 생성한 generator 객체를 호출하면, 함수 안의 코드는 `yield`를 만나기 전까지 실행되고, 첫 번째 값을 반환한다. `for` 구문의 매 loop마다 함수를 다시 반복하고 다음 값을 반환한다. 이러한 반복은 generator가 empty(한 바퀴를 돎) 상태가 될 때까지 이어진다. 또는 loop가 끝나거나 `if/else` 구문을 만족하지 못해도 마찬가지다.

