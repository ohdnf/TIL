# built-in

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