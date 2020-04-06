# django: The web framework for perfectionist with deadlines

- 일반적인 feature(보안 등)이 잘 되어있다.
- `flask`와 다르게 대규모의 application을 만들 때 유용하다.
- Model-View-Controller Model Pattern을 따른다.
    | SW 디자인 패턴 | MVC | Django |
    | ------------- | --- | ------ |
    | 데이터 관리 | Model | **M**odel |
    | 인터페이스(화면) | View | **T**emplate |
    | 중간 관리(상호 동작) | Controller | **V**iew |

## 패키지 설치

- `django==2.1.15` 버전을 사용
    ```bash
    $ pip install django==2.1.15
    ```

## 프로젝트 생성

- `project` == `git`의 `repository` 개념
    ```bash
    $ django-admin startproject [project_name]
    ```

- `project_name` 폴더 안에 동일한 이름의 `project_name` 폴더가 있는 이유
    - 다양한 application을 넣기 위해
    - 앞으로 해당 폴더는 project 폴더라고 지칭

## 앱(Application) 생성

- `pages`라는 앱 생성해보기
    ```bash
    $ python manage.py startapp pages
    ```

- 생성한 앱은 `settings.py` > `INSTALLED_APPS` 리스트에 추가를 해주어야 한다.
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

- 사용자가 요청을 보낼 url `index/`를 `urls.py`에서 정의
    ```python
    #django_intro/urls.py

    ...
    from pages import views

    urlpatterns = [
        # 경로 뒤에 항상 `/`로 닫아준다.
        # 여기서는 index/라는 경로를 만든다.
        path('index/', views.index),
        ...
    ]
    ```

- `views.py`에서 함수 정의
    ```python
    #pages/views.py
    from django.shortcuts import render

    # Create your views here.
    def index(request):
        return render(request, 'index.html')
    ```
    - `render()` 함수를 정의할 때, 항상 첫 번째 인자는 `request`로 작성
        - 내부적으로 요청을 처리할 때, 함수 호출 시 요청 정보가 담긴 객체를 넣어준다.

- `pages/` 앱 안에 `templates/` 폴더 만들고 렌더링할 html 파일 생성
    - 반환할 `html` 파일은 항상 `templates/` 폴더 안에 생성한다.
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
    ```bash
    $ cd django_intro
    ```

- 서버 실행을 위한 `settings.py` 환경 설정
    ```python
    #django_intro/settings.py
    ...
    
    ALLOWED_HOSTS = ['*']

    ...

    LANGUAGE_CODE = 'ko-kr'
    ```

- 서버 실행
    ```bash
    $ python manage.py runserver 8080
    ```

## 앱 만들기

- ㅇㅇ
    ```bash
    $ python manage.py startapp pages
    $ ls
    db.sqlite3  django_intro/  manage.py*  pages/
    ```


## 0406 수업 흐름
- 개발 단계
    1. MVC
    - Model driven design
    - Data Modeling

    2. (urls  ->) View

    3. Controller

- 텔넷
`sudo apt-get install telnetd`

- http 요청-응답 프로세스 추적하는 프로그램
traceroute
tracert

- 오늘 했던 내용
app_name
{% url 'index' %}