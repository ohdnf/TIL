# Django URL

```py
from django.urls import path

from . import views

# app_name은 app마다 namespace를 지정하기 위해 정의한다.
# app_name과 name으로 지정한 urlpattern들은
# `articles:index`와 같은 형식으로 사용 가능하다.
app_name = 'articles'

urlpatterns = [
    path('', views.index, name='index'),
    path('<int:pk>/', views.detail, name='detail'),
    path('create/', views.create, name='create'),
]
```


### next 파라미터 받은 값 POST 요청으로 보내기

[참고](https://stackoverflow.com/questions/16750464/django-redirect-after-login-not-working-next-not-posting)