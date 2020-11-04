# 중첩 함수

> 함수 내에 위치한 또 다른 함수. 바깥에 위치한 함수들과 달리 부모 함수의 변수를 자유롭게 읽을 수 있다. 단일 함수로 문제를 해결해야 하는 코딩 테스트에서 자주 쓰이는 기능이다.



## 예제

```python
def outer_function(t: str):
    text: str = t
    
    def inner_function():
        print(text)
    
    inner_function()

outer_function('Hello!')
```

실행 시 출력은 다음과 같다.

```shell
Hello!
```

> `list()`와 같은 가변 객체인 경우 `append()`, `pop()` 등 여러 가지 연산으로 조작이 가능하다. 하지만 재할당(`=`)이 일어날 경우 참조 ID가 변경되어 별도의 로컬 변수로 선언되므로 주의가 필요하다.



## 연산자 조작

```python
def outer_function(a: List[int]):
    b: List[int] = a
    print(id(b), b)
    
    def inner_function1():
        b.append(4)
        print(id(b), b)
    
    def inner_function2():
        print(id(b), b)
    
    inner_function1()
    inner_function2()

outer_function([1, 2, 3])
```

실행 시 출력

```shell
1952638070400 [1, 2, 3]
1952638070400 [1, 2, 3, 4]
1952638070400 [1, 2, 3, 4]
```



## 재할당

```python
def outer_function(t: str):
    text: str = t
    print(id(text), text)
    
    def inner_function1():
        text = 'World!'
        print(id(text), text)
    
    def inner_function2():
        print(id(text), text)
    
    inner_function1()
    inner_function2()

outer_function('Hello,')
```

실행 시 출력

```shell
1478899091952 Hello,
1478899091824 World!
1478899091952 Hello,
```

> 문자열은 불변 객체이기 때문에 조작할 수 없다. 값을 변경하려면 `text = 'World!'`와 같은 형태로 새롭게 재할당할 수 밖에 없다. `=` 연산자로 변수를 새롭게 할당하는 경우, 기존에 `1478899091952`였던 ID 값이 `1478899091824`로 변경되었다. **중첩 함수 내에서 재할당 시 중첩 함수 내에서만 사용 가능한 새로운 로컬 변수로 선언**되며, 부모 함수에서는 반영되지 않으므로 주의가 필요하다.