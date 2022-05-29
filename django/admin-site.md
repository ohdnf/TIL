# Django Admin Site

> Django admin application은 생성한 모델을 바탕으로 레코드들을 직접 CRUD할 수 있는 사이트다. admin 앱을 사용하기 위해선 모델을 *등록*하고 *superuser를 생성*해야 한다.

## 모델 등록

등록한 모델을 관리할 admin 만들기(model 예시: `Article`)

```python
# posts/admin.py
from django.contrib import admin
# 1. model 불러오기
from .models import Post

# (3. 상세보기 설정)
class PostAdmin(admin.ModelAdmin):
    # Post 관리 페이지에서 보여줄 컬럼들
    list_display = ('id', 'title', 'created_at',)
    # 상세 보기 링크로 지정할 컬럼
    list_display_links = ('title',)
    # 해당 컬럼에 대한 filter 기능 추가
    list_filter = ('created_at')

# 2. Admin을 site에 register
admin.site.register(Post, PostAdmin)
```

## 슈퍼계정 생성

```bash
$ python manage.py createsuperuser
사용자 이름 (leave blank to use 'jp'): admin
이메일 주소: jupyohong7@gmail.com
Password:
Password (again):
Superuser created successfully.
```

## 로그인 후 사용

서버 실행 후 admin 페이지(https://127.0.0.1:8080/admin) 접속

```bash
$ python manage.py runserver
```

> Django Docs
>
> - [The Django admin site](https://docs.djangoproject.com/en/2.2/ref/contrib/admin/)
