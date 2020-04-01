# Basic for Python

파이썬 프로그래밍 문법 정리



## 예외처리

- `if` 문

    ```python
    data = input()
    if not data.isdigit():
    raise TypeError("Must be a number")
    elif data < 0:
        raise ValueError("Must be greater than 0")
    else:
        return 0
    ```

- `try ~ except` 문

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



## string

```python
# 찾기
data_str = "abcdefghidefabc"
print(data_str.find("abc"))
print(data_str.rfind("abc"))
# 삽입
comma_space = ", "
output = comma_space.join(data_str)
print("{0}: {1}".format(type(output), output))
# 대소문자 변경
data_str = "better tomorrow"
data_str_cap = data_str.capitalize()
print("'{0}'".format(data_str_cap))
data_str_low = data_str.lower()
print("'{0}'".format(data_str_low))
data_str_upp = data_str.upper()
print("'{0}'".format(data_str_upp))
# 변경
hello = "Hello, Python!"
hell0 = hello.replace(" ", "")  # 공백 제거
```



## list

### Concatenate items in list to string

- `'[dilimiter]'.join([list])`

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

    - Using `*`(unpack)

        ```python
        # 1부터 100까지 원소 사이에 ', '(쉼표 공백) 넣어서 출력하기
        l = [num for num in range(1, 100) if num % 2 == 1]
        print(*l, sep=', ')
        ```



### 복사

- 얕은 복사

    ```python
    # 주소의 복사
    new_list = old_list
    ```

- 깊은 복사

    ```python
    # 슬라이싱, 가장 빠름
    new_list = old_list[:]

    # 리스트를 추가
    new_list = list()
    new_list.extend(old_list)

    # 새로운 리스트로 복제
    new_list = list(old_list)

    # copy 활용
    import copy
    new_list = copy.copy(old_list)

    # 리스트 함축
    new_list = [e for e in old_list]

    # 리스트 원소까지도 깊은 복사, 가장 느림
    import copy
    new_list = copy.deepcopy(old_list)
    ```



## set

- `data_set = set()`
    
    - `{}`는 딕셔너리의 리터럴이기 때문에 빈 set 객체는 `set()`으로 표현
    
    - 중복된 값을 단일 항목으로 저장해주는 배열 객체

    ```python
    set1 = {1, 2, 3, 4, 5}
    set2 = {3, 4, 5, 6, 7}

    # 교집합
    set1 & set2 == set1.intersection(set2)
    # 합집합
    set1 | set2 == set1.union(set2)
    # 차집합
    set1 - set2 == set1.difference(set2)

    # 항목 추가
    set1.add(6)
    set2.update({8, 9, 10})
    # 항목 제거
    set1.remove(1)
    set2.pop()
    set3 = set2 - set1
    set3.clear()
    # 항목 확인
    # set1이 set2를 전부 포함하는 집합인지 확인
    set1.issuperset(set2)
    # set1이 set2에 전부 포함되는 집합인지 확인
    set1.issubset(set2)
    ```


## dictionary

- `dict() == {}`

    ```python
    # 인덱스X / 중복X / 순서X
    heroes1 = dict(홍길동=20, 이순신=45, 강감찬=35)
    tuple1 = (("홍길동", 20), ("이순신", 45), ("강감찬", 35))
    list1 = [["홍길동", 20], ["이순신", 45], ["강감찬", 35]]
    heroes2 = dict(tuple1)
    heroes3 = dict(list1)

    # 항목 추가
    heroes1["을지문덕"] = 40
    heroes2.update({"신사임당": 50, "유관순": 16})
    # 항목 변경
    heroes3["강감찬"] = 38
    heroes3.update({"이순신": 48, "홍길동": 25,})
    # 항목 제거
    del heroes1["강감찬"]
    heroes1.pop("이순신")
    heroes1.clear()

    # 접근
    print("heroes2.items(): {0}, type(heroes2.items()): {1}".format(heroes2.items(), type(heroes2.items())))
    print("heroes2.keys(): {0}, type(heroes2.keys()): {1}".format(heroes2.keys(), type(heroes2.keys())))
    print("heroes2.values(): {0}, type(heroes2.values()): {1}".format(heroes2.values(), type(heroes2.values())))

    for key in heroes2:
        print("heroes2[{0}]: {1}".format(key, heroes2[key]))

    # 내포
    heroes = {item for item in heroes2.items()}
    print(f"heroes = {type(heroes)}")
    heroes = {item[0]: item[1] for item in heroes2.items()}
    print(heroes)
    ```



## 매개변수

### 명령행 매개변수로부터 인자 값을 읽어오기

- `sys` 모듈 사용

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

- 가장 마지막 매개변수로 지정해야함

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

- 함수 내에서 전역변수에 접근하려면 변수명 앞에 `global` 선언을 해주면 된다.



## lambda식

> `lamda [매개변수]: [반환값]`

    ```python
    def calc(operator_fn, x, y):
        return operator_fn(x, y)

    ret_val = calc(lambda x, y: x + y, 10, 5)
    print(ret_val)

    ret_val = calc(lambda x, y: x - y, 10, 5)
    print(ret_val)
    ```

> 매개변수를 따로 지정해주지 않아도 가능하다

    ```python
    import sys
    input = lambda: sys.stdin.readline().rstrip()
    ```



## 클로저

- 중첩함수 자체를 반환값으로 사용해 정보 은닉 구현

    ```python
    def outer_func():
        id = 0

        def inner_func():
            nonlocal id
            id += 1
            return id

        return inner_func  # 함수 호출이 아닌 함수에 대한 참조를 반환

    make_id = outer_func()
    ```



## 시간 관련 모듈

- `datetime`

    ```python
    from datetime import datetime, timezone, timedelta

    now = datetime.now()
    print("{0}-{1:02}-{2:02} {3:02}:{4:02}:{5:02}".format(now.year, now.month, now.day, now.hour, now.minute, now.second))

    fmt = "%Y{0} %m{1} %d{2} %H{3} %M{4} %S{5}"
    print(now.strftime(fmt).format(*"년월일시분초"))
    ```

- `pytz`

    ```python
    # 타임존 정보 출력
    for tz in list(common_timezones):
        if tz.lower().find("paris") >= 0:
            print(tz)

    tz_utc = timezone(utc.zone)
    tz_seoul = timezone("Asia/Seoul")
    tz_pacific = timezone("US/Pacific")
    tz_paris = timezone("Europe/Paris")

    fmt = "%Y-%m-%d %H:%M:%S %Z%z"

    # UTC 현재 시각
    now_utc = datetime.now(tz_utc)
    print(now_utc.strftime(fmt))

    # Asia/Seoul 타임존
    now_seoul = now_utc.astimezone(tz_seoul)
    print(now_seoul.strftime(fmt))

    # US/Pacific 타임존
    now_pacific = now_seoul.astimezone(tz_pacific)
    print(now_pacific.strftime(fmt))

    # Europe/Paris 타임존
    now_paris = now_pacific.astimezone(tz_paris)
    print(now_paris.strftime(fmt))
    ```



## `random` 모듈

- sample code

    ```python
    from random import random, uniform, randrange, choice, sample, shuffle

    start, stop, step = 1, 45, 2
    print("uniform({0}, {1}) => {2}".format(1.0, 10.0, uniform(1.0, 10.0)))

    print("randrange({0}, {1}) => {2}".format(start, stop, randrange(start, stop)))
    print("randrange({0}) => {1}".format(stop, randrange(stop)))
    print("randrange({0}, {1}, {2}) => {3}".format(start, stop, step, randrange(start, stop, step)))

    data = [1, 2, 3, 4, 5]
    print("choice({0}) => {1}".format(data, choice(data)))
    # print("choices({0}) => {1}".format(data, choices(data, k=2)))
    print("sample({0}) => {1}".format(data, sample(data, k=2)))

    shuffle(data)       # 반환값 없음
    print("shuffled data => {0}".format(data))

    ```



## `@decorator` 함수

- 어떠한 함수를 다른 함수가 실행되기 전에 자동으로 먼저 실행될 수 있도록 해주는 문법

- 파이썬에서는 `functools` 모듈의 `wraps decorator` 함수를 사용해 만들 수 있다.

    ```python
    from functools import wraps
    
    def test_decorator(f):
        @wraps(f)
        def decorated_function(*args, **kwargs):
            print('Decorated Function')
            return f(*args, **kwargs)
        
        return decorated_function

    @test_decorator
    def func():
        print('Calling func function')
    ```

    ```shell
    >>> func()
    Decorated Function
    Calling func function
    ```