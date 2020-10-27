# What is Django: The web framework for perfectionist with deadlines

> Django is a server-side Web Framework using Python,
>
> and helps you to write software that is:
>
> - Complete
> - Versatile
>   - works with any client-side framework
>   - deliver content in any format
> - Secure
>   - manage user accounts and passwords with cryptographic hash function
>     - protection against many risks, such as SQL injections, XSS, CSRF
> - Scalable
>   - share-nothing architecture
>   - can scale for increased traffic by adding hardware at any level
> - Maintainable
>   - DRY principle
>   - MVC pattern
> - Portable
>   - runs on many platforms(Linux, Windows, Mac OS X)
>   - well-supported by many web hosting providers



## What does Django code look like: MVT architecture

> MVT architecture는 Model View Controller architecture와 흡사하다.
>
> | SW 디자인 패턴       | MVC        | Django MTV   |
> | -------------------- | ---------- | ------------ |
> | 데이터 관리          | Model      | **M**odel    |
> | 인터페이스(화면)     | View       | **T**emplate |
> | 중간 관리(상호 동작) | Controller | **V**iew     |

<img src="assets/basic-django.png" alt="Django MTV Model" style="zoom:80%;" />

- `URLs`
  - mapping URLs with view functions
- `View`
  - request handler functions
- `Models`
  - structures of  an application's data
  - provides mechanisms to manage and query records in DB
- `Templates`
  - text file defining the structure or layout of a file with placeholders used to represent actual content



### `urls.py`

> sending the request to the right view

```python
urlpatterns = [
    path('admin/', admin.site.urls),
    path('book/<int:id>/', views.book_detail, name='book_detail'),
    path('catalog/', include('catalog.urls')),
    re_path(r'^([0-9]+)/$', views.best),
]
```



### `views.py`

> handleing the request

```python
# filename: views.py (Django view functions)

from django.http import HttpResponse

def index(request):
    # Get an HttpRequest - the request parameter
    # perform operations using information from the request.
    # Return HttpResponse
    return HttpResponse('Hello from Django!')
```



### `models.py`

> defining data models

```python
# filename: models.py

from django.db import models 

class Team(models.Model): 
    team_name = models.CharField(max_length=40) 

    TEAM_LEVELS = (
        ('U09', 'Under 09s'),
        ('U10', 'Under 10s'),
        ('U11', 'Under 11s'),
        ...  #list other team levels
    )
    team_level = models.CharField(max_length=3, choices=TEAM_LEVELS, default='U11')
```



### `views.py` again

> querying data

```python
## filename: views.py

from django.shortcuts import render
from .models import Team 

def index(request):
    list_teams = Team.objects.filter(team_level__exact="U09")
    context = {'youngest_teams': list_teams}
    return render(request, '/best/index.html', context)
```



### HTML `templates`

> rendering data

```html
## filename: best/templates/best/index.html

<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Home page</title>
</head>
<body>
  {% if youngest_teams %}
    <ul>
      {% for team in youngest_teams %}
        <li>{{ team.team_name }}</li>
      {% endfor %}
    </ul>
  {% else %}
    <p>No teams are available.</p>
  {% endif %}
</body>
</html>
```



### Other things you can do

- Forms
- User authentication and permissions
- Caching
- Administration site
- Serializing data



## 패키지 설치

Django를 사용하기 위해선 Python이 필요하다.

| Django version | Python version             |
| -------------- | -------------------------- |
| `2.2`          | `3.5`, `3.6`, `3.7`        |
| `3.1`          | `3.6`, `3.7`, `3.8`, `3.9` |

Python 설치 후 Django를 설치하기 전 가상환경을 설정해주도록 하자. 가상환경을 설정하는 이유는 의존성 관리를 하기 위해서다.

```bash
# Linux
$ python -m venv venv
$ source venv/bin/activate
(venv)
$ pip install django
```



## 프로젝트 생성

- `django`는 여러 개의 Application을 가진 하나의 Project로 구성

- Project는 git의 Repository 개념
    ```bash
    $ django-admin startproject django_tutorial
    ```

- `django_tutorial` 폴더 안에 동일한 이름의 `django_tutorial` 폴더가 있는 이유

    - 다양한 application을 넣기 위해
    - 상위 폴더는 단순한 container 역할, 하위 폴더는 Python 모듈이다.
    
- 폴더/파일 구조는 다음과 같다

    ```
    django_tutorial/
    	manage.py
    	django_tutorial/
    		__init__.py
    		settings.py
    		urls.py
    		wsgi.py
    ```



## 초기 설정
### `django_tutorial/settings.py` 설정

```python
# settings.py

# 로컬호스트 제외 접속 가능한 URL 지정
# '*': 모든 host/domain 허용
# 추후 white-listing 필요
ALLOWED_HOSTS = ['*']

LANGUAGE_CODE = 'ko-kr'

TIME_ZONE = 'Asia/Seoul'
```



### `.gitignore` 파일 추가

```shell
$ vi .gitignore
```

>  내용은 [gitignore.io](https://www.gitignore.io/api/django) 참고



## 앱(Application) 생성

> Projects vs. apps
>
> 앱(app)은 웹 어플리케이션을 말한다. 예를 들면, 데이터베이스 웹로그 시스템이나 간단한 게시판이다. 프로젝트(project)는 특정 웹사이트를 위한 설정과 어플리케이션의 집합을 말한다. 한 프로젝트는 다수의 앱을 가질 수 있다. 한 앱은 다수의 프로젝트에 속할 수 있다.



### `posts` 앱 생성해보기

```bash
$ python manage.py startapp posts
```

앱 생성 이후 폴더/파일 구조는 다음과 같다

```
django_tutorial/
    manage.py
    django_tutorial/
    posts/
        admin.py
        apps.py
        models.py
        tests.py
        views.py
        __init__.py
        migrations/
```



### 앱 등록

생성한 앱은 `django_tutorial/django_tutorial/settings.py`에 있는 `INSTALLED_APPS` 리스트 변수에 추가를 해주어야 한다.

```python
#django_tutorial/settings.py

...
# Application definition
INSTALLED_APPS = [
    'posts',
    ...
]

...

DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': os.path.join(BASE_DIR, 'db.sqlite3'),
    }
}

...
```



### URL 설정

먼저 프로젝트 폴더 안 `urls.py` 수정

```python
#django_tutorial/urls.py

""" URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/2.2/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""

from django.contrib import admin
from django.urls import path, include		# include 추가

urlpatterns = [
    path('admin/', admin.site.urls),		# 기본으로 생성
    path('posts/', include('posts.urls')),
]
```

사용자 요청을 처리할 앱 `posts`에 `urls.py`를 생성하고 내용 추가

```python
#posts/urls.py
from django.urls import path
from . import views

urlpatterns = [
	# path의 url은 항상 `/`로 닫아준다.
    # 여기서는 index/라는 경로를 만든다.
    path('index/', views.index),
    ...
]
```



### Views(Controller) 설정

`posts/views.py`에서 함수 정의

```python
#posts/views.py
from django.shortcuts import render

# Create your views here.
def index(request):
    return render(request, 'index.html')
```
- `render()` 함수를 정의할 때, 항상 첫 번째 인자는 `request`로 작성
    - 내부적으로 요청을 처리할 때, 함수 호출 시 요청 정보가 담긴 객체를 넣어준다.



### Settings for serving Static Files

정적 파일을 사용하기 위한 설정 추가

```python
#django_tutorial/urls.py
from django.conf import settings
from django.conf.urls.static import static

urlpatterns += static(settings.STATIC_URL, document_root=settings.STATIC_ROOT)
```



### Templates(View) 설정

`posts/` 앱 안에 `templates/` 폴더 만들고 렌더링할 html 파일 생성

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



## DB migration

> Django model을 생성, 수정해 DB 구조(스키마)가 바뀔 때마다 migration이 필요하다.
> 프로젝트를 초기화했을 때도 마찬가지다.
>
> migration은 다음 두 명령어로 실행한다.

```shell
$ python manage.py makemigrations
$ python manage.py migrate
```



## 서버 실행

- 반드시 서버 실행 시 실행되는 디렉토리를 확인할 것
  
- `manage.py`가 있는 경로에서 서버를 실행한다!
  
    ```bash
    $ cd ~/django_tutorial
    $ python manage.py runserver
    ```

