# Static

Django에서 JavaScript, CSS, image 파일과 같은 정적 파일을 관리하는 폴더

## 파일 구조

```txt
django_pjt/
    articles/
        migrations/
        static/
            articles/
                images/
                stylesheets/
                    style.css
        templates/
            articles/
                form.html
        __init__.py
        admin.py
        apps.py
        models.py
        tests.py
        urls.py
        views.py
    django_pjt/
        __init__.py
        settings.py
        ulrs.py
        wsgi.py
    static/
        bootstrap/

    templates/
        base.html
    db.sqlite3
    manage.py
    README.md
```

## 프로젝트 폴더와 같은 위치에 `static` 폴더 만들기

- `settings.py` 설정

  ```python
  # settings.py

  ...
  # Static files (CSS, JavaScript, Images)
  # https://docs.djangoproject.com/en/2.1/howto/static-files/

  # STATIC_URL은 외부에서 접속 가능하도록 설정
  # domain_url/STATIC_URL/serving_되는_static_파일_경로
  # 와 같은 형태로 html source에 표현
  STATIC_URL = '/static/'

  # app 디렉토리가 아닌 static 폴더 지정
  STATICFILES_DIRS = [
      os.path.join(BASE_DIR, 'static'),
  ]
  ```

## 앱 폴더 안에 `static` 폴더 만들기

- 향후 다른 앱에서도 `static` 폴더를 만들기 위해서는 각 앱의 `static` 폴더 안에 앱 이름의 폴더를 만들어주는 것이 바람직하다(`templates`처럼).

- `templates/base.html`

  ```html
  <!DOCTYPE html>
  {% load static %}
  <html lang="ko">
    <head>
      <meta charset="UTF-8" />
      <meta name="viewport" content="width=device-width, initial-scale=1.0" />
      <title>Django Project</title>
      <!-- Bootstrap CDN을 Static 파일로 교체 -->
      <!-- <link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/css/bootstrap.min.css" integrity="sha384-Vkoo8x4CGsO3+Hhxv8T/Q5PaXtkKtu6ug5TOeNV6gBiFeWPGFN9MuhOf23Q9Ifjh" crossorigin="anonymous"> -->
      <link
        rel="stylesheet"
        href="{% static 'bootstrap/bootstrap.min.css' %}"
      />
    </head>
    <body>
      <!-- Navigation -->
      {% block body %} {% endblock %}

      <script
        src="https://code.jquery.com/jquery-3.4.1.slim.min.js"
        integrity="sha384-J6qa4849blE2+poT4WnyKhv5vZF5SrPo0iEjwBvKU7imGFAV0wwj1yYfoRSJoZ+n"
        crossorigin="anonymous"
      ></script>
      <script
        src="https://cdn.jsdelivr.net/npm/popper.js@1.16.0/dist/umd/popper.min.js"
        integrity="sha384-Q6E9RHvbIyZFJoft+2mJbHaEWldlvI9IOYy5n3zV9zzTtmI3UksdQRVvoxMfooAo"
        crossorigin="anonymous"
      ></script>
      <!-- Bootstrap CDN을 Static 파일로 교체 -->
      <!-- <script src="https://stackpath.bootstrapcdn.com/bootstrap/4.4.1/js/bootstrap.min.js" integrity="sha384-wfSDF2E50Y2D1uUdj0O3uMBJnjuUD4Ih7YwaYd1iqfktj0Uod8GCExl3Og8ifwB6" crossorigin="anonymous"></!-->
      -->
      <script src="{% static 'bootstrap/bootstrap.min.js' %}"></script>
    </body>
  </html>
  ```

- `articles/templates/articles/form.html`

  ```html
  <!-- extends DTL은 항상 맨 위에 있어야 한다. load static은 밑에 써준다. -->
  {% extends 'base.html' %} {% load static %} ...
  ```
