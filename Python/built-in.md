## __doc__
python 파일 내 설명문(docstring)을 제공해주는 객체

* 예시
```
#mymodule.py
"""This is the module docstring."""

def f(x):
    """This is the function docstring."""
    return 2 * x
```

command line에서
```
>>> import mymodule
>>> mymodule.__doc__
'This is the module docstring.'
>>> mymodule.f.__doc__
'This is the function docstring.'
```


## sys.argv
python 파일을 실행시켰을때 command line에서 넘겨주는 인자를 저장해주는 객체

* 사용 예시
```
import sys
program_name = sys.argv[0]
arguments = sys.argv[1:]
count = len(arguments)
```

다른 예시
```
# test.py

import sys

print("sys.argv => {0} {1}".format(type(sys.argv), sys.argv))

for i, val in enumerate(sys.argv):
    print("sys.argv[{0}] => {1}".format(i, val))

```

터미널창에서 인자를 space로 구분하여 전달
```
$ python test.py 1 2 3
sys.argv => <class 'list'> ['test.py', '1', '2', '3']
sys.argv[0] => 200115.py
sys.argv[1] => 1
sys.argv[2] => 2
sys.argv[3] => 3
```

## property
property 데코레이터는 클래스 메소드가 클래스 변수에 접근할 수 있도록 도와준다.

```
class property(fget=None, fset=None, fdel=None, doc=None)
```

참조 링크:
https://docs.python.org/3/library/functions.html#property

* 사용 예시 1: property()
```
class C:
    def __init__(self):
        self._x = None

    def getx(self):
        return self._x

    def setx(self, value):
        self._x = value

    def delx(self):
        del self._x

    x = property(getx, setx, delx, "I'm the 'x' property.")
```

* x를 호출하면 getx()를, x를 설정하면 setx()를, x를 삭제하면 delx() 함수를 각각 자동으로 호출하고, x에 대한 doc(x.__doc__)을 호출하면 property 마지막 인자로 넘어간 str을 출력한다.



* 사용 예시 2: @property

```
class Parrot:
    def __init__(self):
        self._voltage = 100000

    @property
    def voltage(self):
        """Get the current voltage."""
        return self._voltage
```

* 더 알아보기: 
https://www.programiz.com/python-programming/property

```
class Celsius:
    def __init__(self, temperature = 0):
        self._temperature = temperature

    def to_fahrenheit(self):
        return (self.temperature * 1.8) + 32

    @property
    def temperature(self):
        print("Getting value")
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if value < -273:
            raise ValueError("Temperature below -273 is not possible")
        print("Setting value")
        self._temperature = value
```