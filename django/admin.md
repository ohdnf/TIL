# admin

- 등록한 모델을 관리할 admin 만들기(model 예시: `Article`)

    ```py
    # articles/admin.py
    from django.contrib import admin
    from .models import Article

    class ArticleAdmin(admin.ModelAdmin):
        list_display = ('title', 'content',)
        list_display_links = ['title',]
        list_filter = ['created_at',]

    admin.site.register(Article, ArticleAdmin)
    ```

- Admin 유저 생성
    ```bash
    $ python manage.py createsuperuser
    $ admin ID:
    $ email:
    $ password:
    $ password confirm:
    ```

- 서버 실행 후 admin 페이지(https://127.0.0.1:8080/admin) 접속
    ```bash
    $ python manage.py runserver 8080
    ```
