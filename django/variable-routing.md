# Variable Routing

> URL의 특정 위치의 값을 변수로 활용

## 1. `urls.py`

```py
# django_intro.urls.py
path('hi/<str:name>/', views.hi),
path('add/<int:a>/<int:b>/', views.add),
```

- Path converters
  - `str`: 띄어쓰기 없는 string
  - `int`: 0 또는 양의 정수
  - `slug`: `-`(hyphen) 또는 `_`(underscore)로 연결된 string
  - `uuid`: `-`(dash)와 소문자로 구성된 UUID
  - `path`: `/`을 포함한 기본 경로

## 2. `views.py`

```py
# pages/views.py
def hi(request, name):
    context = {
        'name': name,
    }
    return render(request, 'hi.html', context)

def add(request, a, b):
    result = a + b
    context = {
        'a': a,
        'b': b,
        'result': result,
    }
    return render(request, 'add.html', context)
```

## 3. `templates/`

```html
<!-- pages/templates/hi.html -->
<p>Hello, {{ name }}</p>
```

```html
<!-- pages/templates/add.html -->
<p>{{a}} + {{b}} = {{result}}</p>
```
