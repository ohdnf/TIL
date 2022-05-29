# How Django URL mappers, views and models work

랜딩 페이지를 만들어보면서 Django의 데이터 흐름이 어떻게 구성되는지 알아보자

- URL mapper를 통해 지원하는 URL을 view 함수에 연결
- View 함수는 DB에서 필요한 자료를 찾아 HTML 페이지를 만들어 요청자에게 반환
- Templates는 views에서 데이터를 HTML 페이지에 렌더링할 때 사용

## Defining the resource URLs

일반적으로 사용하는 URL 예시는 다음과 같다.

- `posts/`
  - 모든 게시글을 보여주는 목록(index) 페이지
- `posts/<id>`
  - id에 해당하는 게시글의 상세(detail) 페이지

```python
from django.urls import path
from . import views

# app_name은 app마다 namespace를 지정하기 위해 정의한다.
# app_name과 name으로 지정한 urlpattern들은
# `posts:index`와 같은 형식으로 사용 가능하다.
app_name = 'posts'

urlpatterns = [
    path('', views.index, name='index'),
    # URL에서 pk의 데이터 형식을 정수(int)로 지정한다.
    path('<int:pk>/', views.detail, name='detail'),
]
```

## View functions

View는 HTTP 요청을 처리하는 함수로, DB에서 필요한 데이터를 가져와 HTML 템플릿 파일에 렌더링하거나 요청된 자료구조로 변환해 반환해준다.

`urls.py`에서 매핑한 views 함수를 만들어보자.

```python
from django.shortcuts import render

from .models import Post


def index(request):
    posts = Post.objects.all()
    context = {
        'posts': posts
    }
    return render(request, 'index.html', context)
```

## Templates

> HTML 페이지의 레이아웃, 구조를 정의하는 파일

Django는 기본적으로 앱 안의 `templates`라는 이름의 폴더에서 렌더링할 템플릿을 찾는다. 찾지 못한다면 `TemplateDoesNotExist at /posts/`와 같은 에러를 발생시킨다.

### Extending templates

기본 뼈대 구조인 base template 파일을 만들고 그 안에 block template를 만들어 확장(extends)하는 방식으로 코드의 중복을 피할 수 있다.

#### `base.html`

```html
<!-- base.html -->
<!DOCTYPE html>
<html lang="en">
  <head>
    {% block title %}
    <title>Local Library</title>
    {% endblock %}
  </head>
  <body>
    {% block sidebar %}<!-- insert default navigation text for every page -->{%
    endblock %} {% block content %}<!-- default content text (typically empty) -->{%
    endblock %}
  </body>
</html>
```

#### `index.html`

```html
{% extends "base.html" %} {% block content %}
<h1>Local Library Home</h1>
<p>
  Welcome to LocalLibrary, a website developed by
  <em>Mozilla Developer Network</em>!
</p>
{% endblock %}
```

### next 파라미터 받은 값 POST 요청으로 보내기

[참고](https://stackoverflow.com/questions/16750464/django-redirect-after-login-not-working-next-not-posting)
