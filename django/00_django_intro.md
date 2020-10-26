# Django: The web framework for perfectionist with deadlines

Server-side Web Framework using Python

![Django MTV Model](assets/basic-django.png)



## 특징

- 일반적인 feature(보안, 유지보수 등)이 잘 되어있다.

- `flask`와 다르게 대규모의 application을 만들 때 유용하다.

- Model-View-Controller Model Pattern을 따른다: **MTV**
    | SW 디자인 패턴 | MVC | Django |
    | ------------- | --- | ------ |
    | 데이터 관리 | Model | **M**odel |
    | 인터페이스(화면) | View | **T**emplate |
    | 중간 관리(상호 동작) | Controller | **V**iew |



## 패키지 설치

- `django==2.1.15` 버전 사용
  
    ```bash
    $ pip install django==2.1.15
    ```



## 프로젝트 생성

- `django`는 여러 개의 Application을 가진 하나의 Project로 구성
- Project는 git의 Repository 개념
    ```bash
    $ django-admin startproject 프로젝트명
    ```

- `project_name` 폴더 안에 동일한 이름의 `project_name` 폴더가 있는 이유

    - 다양한 application을 넣기 위해
    - 앞으로 상위 폴더는 project 폴더라고 지칭(하위 폴더는 앱)



## 초기 설정
- `project_name/settings.py` 설정
    ```python
    # settings.py

    # 로컬호스트 제외 접속 가능한 URL 지정
    # '*': 모든 host/domain 허용
    # 추후 white-listing 필요
    ALLOWED_HOSTS = ['*']

    LANGUAGE_CODE = 'ko-kr'

    TIME_ZONE = 'Asia/Seoul'
    ```
    
- `.gitignore` 파일 추가
    ```shell
    $ vi .gitignore
    ```

    - 내용은 [gitignore.io](https://www.gitignore.io/api/django) 참고



## 전체 흐름

URL 설정 => Views (=> Templates)



## 앱(Application) 생성

> Projects vs. apps
>
> 앱(app)은 웹 어플리케이션을 말한다. 예를 들면, 데이터베이스 웹로그 시스템이나 간단한 게시판이다. 프로젝트(project)는 특정 웹사이트를 위한 설정과 어플리케이션의 집합을 말한다. 한 프로젝트는 다수의 앱을 가질 수 있다. 한 앱은 다수의 프로젝트에 속할 수 있다.

### `pages`라는 앱 생성해보기

```bash
$ python manage.py startapp pages
```

### 앱 등록

생성한 앱은 `settings.py` > `INSTALLED_APPS` 리스트에 추가를 해주어야 한다.

```python
#django_intro/settings.py

...
# Application definition
INSTALLED_APPS = [
    'pages',
    ...
]

...
```

### URL 설정

사용자가 요청을 보낼 url `index/`를 `urls.py`에서 정의

```python
#django_intro/urls.py

...
from pages import views

urlpatterns = [
    # path의 url은 항상 `/`로 닫아준다.
    # 여기서는 index/라는 경로를 만든다.
    path('index/', views.index),
    ...
]
```

### Views(Controller) 설정

`views.py`에서 함수 정의

```python
#pages/views.py
from django.shortcuts import render

# Create your views here.
def index(request):
    return render(request, 'index.html')
```
- `render()` 함수를 정의할 때, 항상 첫 번째 인자는 `request`로 작성
    - 내부적으로 요청을 처리할 때, 함수 호출 시 요청 정보가 담긴 객체를 넣어준다.

### Templates(View) 설정

`pages/` 앱 안에 `templates/` 폴더 만들고 렌더링할 html 파일 생성

>  반환할 `html` 파일은 항상 `templates/` 폴더 안에 생성한다.

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="X-UA-Compatible" content="ie=edge">
    <title>My First Django Project</title>
</head>
<body>
    <h1>Hello, world!</h1>
</body>
</html>
```



## 서버 실행

- 반드시 서버 실행 시 실행되는 디렉토리를 확인할 것
  
- `manage.py`가 있는 경로에서 서버를 실행한다!
  
    ```bash
    $ cd ~/django_intro
    $ python manage.py runserver 8080
    ```

