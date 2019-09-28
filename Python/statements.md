#assert

### 조건이 틀리면 AssertionError를 발생시킨다.

```
assert condition, "The condition is False"
```
위와 같은 형태로 사용하며 아래 코드와 동일하다.
```
if not condition:
    raise AssertionError()
```
사실 위의 코드는 아래와 같은데,
```
if __debug__:
    if not expression: raise AssertionError
```
command line에서 스크립트를 실행할 때 -O 옵션을 주면 __debug__ 빌트인 변수가 False가 된다.