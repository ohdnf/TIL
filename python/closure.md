# Closure

> 클로저는 일급 객체 함수의 개념을 이용하여 스코프(scope)에 묶인 변수를 바인딩하기 위한 일종의 기술입니다.

```python
def outer_func():	# 1
    msg = 'hi'	# 3
    
    def inner_func():	# 4
        print(msg)	# 6
       
    return inner_func()	# 5

outer_func()	# 2
```

실행 프로세스

1. `#1`에서 정의된 함수 `outer_func`를 `#2`에서 호출
2. `outer_func`가 실행된 후, `msg`에 `'hi'`라는 문자열 할당(`#3`)
3. `#4`에서 `inner_func`를 정의하고 `#5`에서 호출, 동시에 반환
4. `#6`에서 `msg` 변수를 참조해 출력
   - 여기서 `msg`는 `inner_func`안에서 정의되지 않았지만 `inner_func` 안에서 사용되기 때문에 **자유 변수(free variable)**라고 부릅니다.



`inner_func`를 실행하지 않고 함수 오브젝트로 반환하면 출력되는 값은 없습니다. 반환값을 새로운 변수에 할당하고 실행하면 `hi`가 출력됩니다. 아래 코드를 통해 클로저가 함수의 자유 변수 값을 어디에 저장하는지 알아봅니다.

```python
def outer_func():	# 1
    msg = 'hi'	# 3
    
    def inner_func():	# 4
        print(msg)	# 6
       
    return inner_func	# 5

outer_func()	# 아무 것도 출력되지 않음
my_func = outer_func()	# 2

my_func()	# hi 출력

print('-'*30)
print(my_func)
print('-'*30)
print(dir(my_func))
print('-'*30)
print(type(my_func.__closure__))
print('-'*30)
print(type(my_func.__closure__[0]))
print('-'*30)
print(dir(type(my_func.__closure__[0])))
print('-'*30)
print(my_func.__closure__[0].cell_contents)
```

```shell
hi
------------------------------
<function outer_func.<locals>.inner_func at 0x0000017E4B618E50>
------------------------------
['__annotations__', '__call__', '__class__', '__closure__', '__code__', '__defaults__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__get__', '__getattribute__', '__globals__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__kwdefaults__', '__le__', '__lt__', '__module__', '__name__', '__ne__', '__new__', '__qualname__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__']
------------------------------
<class 'tuple'>
------------------------------
<class 'cell'>
------------------------------
['__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', 'cell_contents']
------------------------------
hi
```

