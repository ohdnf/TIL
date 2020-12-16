# First Class Function

> 일급 함수란 프로그래밍 언어가 함수를 일급 시민(First class citizen)으로 취급하는 것을 뜻합니다. 이것은 해당 언어에서 함수를 
>
> - 다른 함수의 인자로 넘기거나, 
> - 결과값으로 반환하거나, 
> - 데이터 구조에 변수에 할당할 수 있다
> - (익명 함수를 지원해야 한다는 조건을 덧붙이기도 한다)
>
> 는 것을 의미합니다. 

일급 함수의 개념은 아래와 같이 `logger()`와 같은 wrapper 함수 안에서 기존에 정의한 함수나 모듈을 재사용할 때 편리합니다.

```python
def logger(msg):
    
    def log_msg():
        print(f"Log: {msg}")
        
    return log_msg

log_hi = logger("hi")
print(log_hi)
log_hi()

del logger

try:
    print(logger)
except NameError as e:
    print(e)
```

```shell
<function logger.<locals>.log_msg at 0x0000019B30998EE0>
Log: hi
name 'logger' is not defined
Log: hi
```

`logger` 함수를 글로벌 네임스페이스에서 삭제한 후에도 `msg` 변수에 할당됐던 `hi` 값이 참조되고 있습니다. `log_msg`와 같은 함수를 **클로저(Closure)**라고 부르며 클로저는 다른 함수의 지역변수를 그 함수가 종료된 이후에도 기억할 수 있습니다.