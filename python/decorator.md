# Decorator

> 인자로 함수를 받아 명령을 추가한 뒤 다시 함수의 형태로 반환하는 함수
>
> - 함수의 내부를 수정하지 않고 기능에 변화를 주고 싶을 때
> - 함수의 전처리나 후처리가 필요할 때
> - 반복을 줄이고 메소드나 함수의 책임을 확장할 때
>
> 사용합니다.

## 구조

데코레이터는 클로저와 유사합니다. 다만, **함수를 다른 함수의 인자로 전달**한다는 점이 다릅니다.

```python
def decorator_func(original_func):	# 1
    def wrapper_func():		# 5
        return original_func()	# 7
    return wrapper_func		# 6

def display():	# 2
    print("화면을 출력합니다.")		# 8

decorated_display = decorator_func(display)		# 3
decorated_display()		# 4
```

```shell
화면을 출력합니다.
```



`@` 기호를 사용한 데코레이터 구문이 일반적입니다. 인수를 받는 함수와 그렇지 않은 함수를 decorate할 수 있는 데코레이터 함수를 만들어 봅니다.

```python
def decorator_func(original_func):
    def wrapper_func(*args, **kwargs):
        print("데코레이터 함수 내부...")
        return original_func(*args, **kwargs)
    return wrapper_func

@decorator_func
def display_1():
    print("화면 출력 1")

@decorator_func
def display_2(year, month, day):
    print(f"화면 출력 2: {year}-{month}-{day}")
    
# display_1 = decorator_func(display_1)
# display_2 = decorator_func(display_2)

display_1()
display_2(2020, 12, 16)
```

```shell
데코레이터 함수 내부...
화면 출력 1
데코레이터 함수 내부...
화면 출력 2: 2020-12-16
```



클래스 형식을 사용할 수도 있습니다.

```python
# def decorator_func(original_func):
#     def wrapper_func(*args, **kwargs):
#         print("데코레이터 함수 내부...")
#         return original_func(*args, **kwargs)
#     return wrapper_func

class DecoratorClass:
    def __init__(self, original_func):
        self.original_func = original_func
        
    def __call__(self, *args, **kwargs):
        print("데코레이터 클래스 내부...")
        return self.original_func(*args, **kwargs)

@DecoratorClass
def display_1():
    print("화면 출력 1")

@DecoratorClass
def display_2(year, month, day):
    print(f"화면 출력 2: {year}-{month}-{day}")

display_1()
display_2(2020, 12, 16)
```

```shell
데코레이터 클래스 내부...
화면 출력 1
데코레이터 클래스 내부...
화면 출력 2: 2020-12-16
```



일반적으로 데코레이터는

- 로그를 남기거나
- 유저의 로그인 상태 등을 확인하여 후처리(로그인하지 않았다면 로그인 페이지로 리다이렉트)
- 프로그램의 성능을 테스트(스크립트가 실행되는 시간을 측정)

하기 위해 많이 사용합니다.

```python
import datetime
import time


def my_logger(original_func):
    import logging
    logging.basicConfig(filename=f'{original_func.__name__}.log', level=logging.INFO)
    
    def wrapper(*args, **kwargs):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
        logging.info(f'[{timestamp}] 실행결과 args: {args}, kwargs: {kwargs}')
        return original_func(*args, **kwargs)
    
    return wrapper


@my_logger
def display_info(year, month, day):
    time.sleep(1)
    print(f'화면 출력: {year}-{month}-{day}')

display_info(2002, 6, 18)
```

```shell
화면 출력: 2002-6-18
display_info 함수 실행 시간: 1.0171144008636475 sec
```

로그 파일 `display_info.log` 확인

```log
INFO:root:[2020-12-16 22:00] 실행결과 args: (2002, 6, 18), kwargs: {}
```



만약에 `my_timer`라는 데코레이터 함수를 하나 더 만들고 `display_info` 함수를 데코레이팅하고 있는 `my_logger` 밑이나 위에 넣으면 어떻게 될까요? 두 데코레이터 모두 `wrapper` 함수를 반환하고 있기 때문에 반환된 함수를 받는 데코레이터는 `original_function`을 이전 데코레이터에서 반환한 `wrapper` 함수로 처리하게 됩니다.

```python
import datetime
import time


def my_logger(original_func):
    import logging
    logging.basicConfig(filename=f'{original_func.__name__}.log', level=logging.INFO)
    
    def wrapper(*args, **kwargs):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
        logging.info(f'[{timestamp}] 실행결과 args: {args}, kwargs: {kwargs}')
        return original_func(*args, **kwargs)
    
    return wrapper


def my_timer(original_func):
    
    def wrapper(*args, **kwargs):
        t1 = time.time()
        result = original_func(*args, **kwargs)
        t2 = time.time() - t1
        print(f'{original_func.__name__} 함수 실행 시간: {t2} sec')
        return result
    
    return wrapper

    
@my_logger
@my_timer
def display_info(year, month, day):
    time.sleep(1)
    print(f'화면 출력: {year}-{month}-{day}')

display_info(2002, 6, 18)
```

출력은 제대로 나오지만, `display_info.log` 파일 대신 `wrapper.log` 파일이 생성됩니다.

```log
INFO:root:[2020-12-16 22:01] 실행결과 args: (2002, 6, 18), kwargs: {}
```



복수의 데코레이터를 한 함수에 중첩(stack)해서 사용하기 위해서는 `functools` 모듈의 `wraps` 데코레이터를 사용할 수 있습니다.

```python
from functools import wraps
import datetime
import time


def my_logger(original_func):
    import logging
    logging.basicConfig(filename=f'{original_func.__name__}.log', level=logging.INFO)
    
    @wraps(original_func)
    def wrapper(*args, **kwargs):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
        logging.info(f'[{timestamp}] 실행결과 args: {args}, kwargs: {kwargs}')
        return original_func(*args, **kwargs)
    
    return wrapper


def my_timer(original_func):
    
    @wraps(original_func)
    def wrapper(*args, **kwargs):
        t1 = time.time()
        result = original_func(*args, **kwargs)
        t2 = time.time() - t1
        print(f'{original_func.__name__} 함수 실행 시간: {t2} sec')
        return result
    
    return wrapper

    
@my_logger
@my_timer
def display_info(year, month, day):
    time.sleep(1)
    print(f'화면 출력: {year}-{month}-{day}')

display_info(2020, 12, 16)
```

```shell
화면 출력: 2020-12-17
display_info 함수 실행 시간: 1.0251619815826416 sec
```

`display_info.log`

```log
INFO:root:[2020-12-16 22:00] 실행결과 args: (2002, 6, 18), kwargs: {}
INFO:root:[2020-12-16 22:09] 실행결과 args: (2020, 12, 16), kwargs: {}
```

