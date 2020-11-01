# python-decouple

> `.env`나 `settings.ini`와 같은 설정 파일을 파싱해주는 파이썬 라이브러리



## 사용법

### 설치

```shell
$ pip install python-decouple
```



### 환경 변수 설정

#### `.env` 작성

```
DJANGO_SECRET_KEY=A8a&SnIS^SaSjihoAS*&ajh2!SB&%S4ha
```



### Django 설정 파일에서 읽어오기(예시)

#### `settings.py`

```python
import os
from decouple import config

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

SECRET_KEY = config('DJANGO_SECRET_KEY')
```

