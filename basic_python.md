# Basic for Python

파이썬 프로그래밍 문법 정리



## 예외처리

* `if` 문

  ```python
  data = input()
  if not data.isdigit():
  	raise TypeError("Must be a number")
  elif data < 0:
      raise ValueError("Must be greater than 0")
  else:
      return 0
  ```

* `try ~ except` 문

  ```python
  try:
  	
  except Exception as ex:
  	# 예외가 발생했을 때 처리
      print("{0}: {1}".format(type(ex), ex))
  else:
      # 예외가 발생하지 않았을 때 실행
  finally:
  	# 예외 발생과 상관없이 실행
  ```



## 리스트

### Concatenate items in list to string

#### `'[dilimiter]'.join([list])`

1. type(item) == str

    ```python
    data = ['h', 'e', 'l', 'l', 'o']
    print('-'.join(data))
    # 'h-e-l-l-o'
    ```

2. type(item) == int

    ```python
    data = [1, 2, 3, 4, 5]
    print('-'.join(map(str, data))))
    # '1-2-3-4-5'
    ```

#### Using `*`(unpack)

1부터 100까지 원소 사이에 ', '(쉼표 공백) 넣어서 출력하기

```python
l = [num for num in range(1, 100) if num % 2 == 1]
print(*l, sep=', ')
```



## 매개변수

### 명령행 매개변수로부터 인자 값을 읽어오기

`sys` 모듈 사용

```shell
$ touch test.py
$ code test.py
```

```python
#test.py
import sys

print("sys.argv => {0} {1}".format(type(sys.argv), sys.argv))

for i, val in enumerate(sys.argv):
    print("sys.argv[{0}] => {1}".format(i, val))

print("/n")
```

```shell
$ python test.py 1 2 3 
sys.argv => <class 'list'> ['test.py', '1', '2', '3']
sys.argv[0] => test.py
sys.argv[1] => 1
sys.argv[2] => 2
sys.argv[3] => 3
```

### 사용자 함수에서 가변형 매개변수 사용하기: `def func(*args):`

가장 마지막 매개변수로 지정해야함

```python
def calc_sum(precision, *params):
    if precision == 0:
        total = 0
    elif 0 < precision < 1:
        total = 0.0
    for val in params:
        total += val
    return total
```

### 키워드 언팩 연산자: `def func(**kwargs)`

```python
def use_keyword_arg_unpacking(**params):
    for k in params.keys():
        print("{0}: {1}".format(k, params[k]))

print("use_keyword_arg_unpacking()의 호출")
use_keyword_arg_unpacking(a=1, b=2, c=3)
```

- 기본 값을 가지는 매개변수는 일반 매개변수 앞에 위치할 수 없다
- 이름이 같을 경우 지역변수 다음에 전역변수에 접근하므로 전역변수가 접근하지 못할 경우 발생 가능
- 함수 내에서 전역변수에 접근하려면 변수명 앞에 global 선언을 해주면 된다.



## lambda식

`lamda [매개변수]: [반환값]`

```python
def calc(operator_fn, x, y):
    return operator_fn(x, y)

ret_val = calc(lambda x, y: x + y, 10, 5)
print(ret_val)

ret_val = calc(lambda x, y: x - y, 10, 5)
print(ret_val)
```

매개변수를 따로 지정해주지 않아도 가능하다

```python
import sys
input = lambda: sys.stdin.readline().rstrip()
```



## 클로저

중첩함수 자체를 반환값으로 사용해 정보 은닉 구현

```python
def outer_func():
    id = 0

    def inner_func():
        nonlocal id
        id += 1
        return id

    return inner_func       # 함수 호출이 아닌 함수에 대한 참조를 반환

make_id = outer_func()
```