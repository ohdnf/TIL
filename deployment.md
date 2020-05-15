# 배포



## What

우리는 서버컴퓨터에서 요청과 응답을 처리할 프로그램을 개발한다.


## When

While alive:

    분석/계획/설계 > 개발(Development) > 테스트(TDD) > **배포(배치, Deployment)** > 운영(Operation)
                개발(Development Stage)            |    제품 출시 및 운영(Production Stage)

## Who & Where

누가, 어디에 배치(설치)하는가?

0. 제공자가 사용자가 컴퓨터에 

1. Download == Native App: 사용자가 사용자 컴퓨터에

    > `.dmg`, `.exe`, `.iso`, `$ brew`, `$ apt`, `$ choco`

2. Web App: 제공자가 제공자 컴퓨터에

## Where

- 직접 산 컴퓨터 vs. 빌린 컴퓨터(Cloud!)

- 컴퓨터 빌려주는 플랫폼들...

    - AWS
    - MS Azure
    - Google Cloud

## How

- AWS 구조
    - DNS(Route53): Routing
    - Main Computer(EC2): Python & Django
    - DB(RDS): MySQL, pg, etc.. + Disk(S3): static, media files..

- CI(Continuous Integration) Server

## Why - In reality

프로그램을 개발하는 이유?

> Real Artists Ship
>           - Steve Jobs
> 
> 모든 작품은 세상에 공개(Ship)될 때, 비로소 완성되며 동시에 생(Life)을 시작한다.
> 완성은 시작보다 어려운 일이며, 공개는 겁나는 일이다.
>
> 100개의 걸출한 미완의 습작보다 1개의 완성된 졸작이 훨씬 더 큰 의미를 갖는다.



## Heroku Deployment

### 참조
- [MDN 공식 문서](https://developer.mozilla.org/en-US/docs/Learn/Server-side/Django/Deployment)
- [강동주 선생님 GitLab](https://lab.ssafy.com/john/heroku_deploy)

### I. Heroku

1. [Heroku 가입](http://heroku.com/)

2. Heroku CLI 로그인

    - [설치](https://devcenter.heroku.com/articles/heroku-cli#download-and-install)

    - CLI에서 heroku 설치 확인 후 로그인

        ```bash
        $ heroku --version
        $ heroku login -i
        ```

### II. Django 프로젝트 생성 및 배포 설정

1. 프로젝트 생성

    ```bash
    $ mkdir django_pjt
    $ cd django_pjt
    ```

2. 가상(독립) 환경 설정

    - 가상환경 생성 및 실행

        ```bash
        $ python -m venv venv
        $ source venv/bin/activate      # Linux OS
        $ source venv/Scripts/activate  # Windows OS
        ```

3. 패키지 설치

    - Django 관련 및 배포를 위한 패키지 설치
    
        ```bash
        $ pip install django==2.1.15 dj-database-url gunicorn whitenoise
        ```

4. Django 프로젝트 생성

    - Django 프로젝트 생성

        ```bash
        $ django-admin startproject instagram .
        ```

### III. (**중요**) 배포 설정

1. `Procfile`

    - `Procfile` 파일 생성 후 다음 내용 추가

        ```procfile
        web: gunicorn instagram.wsgi --log-file -
        ```

    - `web: gunicorn <프로젝트명>.wsgi --log-file -`
    - (_주의_) `<프로젝트명>`엔 반드시 프로젝트 폴더명을 적어야 함

2. `requirements.txt`

    - 패키지 리스트 requirements 파일 생성

        ```shell
        $ pip freeze > requirements.txt
        ```

    - `requirements.txt`에 `psycopg2` 패키지 추가

        ```txt
        dj-database-url==0.5.0
        Django==2.1.15
        ...
        psycopg2==2.7.7
        ```

3. `runtime.txt`

    - `runtime.txt` 파일 생성 후 다음 내용 추가

        ```txt
        python-3.7.3
        ```
    
    - 현재 `Python` 버전과 일치해야 함

        ```shell
        $ python --version
        Python 3.7.3
        ```
### IV. 세부 설정

1. `settings.py`

    - `settings.py` 최하단에 다음 내용 추가(Database 관련)

        ```py
        # Heroku: Update database configuration from $DATABASE_URL.
        import dj_database_url
        db_from_env = dj_database_url.config(conn_max_age=500)
        DATABASES['default'].update(db_from_env)
        ```

    - `settings.py` 최하단에 다음 내용 추가(static 관련)

        ```py
        # Static files (CSS, JavaScript, Images)
        # https://docs.djangoproject.com/en/2.0/howto/static-files/

        # The absolute path to the directory where collectstatic will collect static files for deployment.
        STATIC_ROOT = os.path.join(BASE_DIR, 'staticfiles')

        # The URL to use when referring to static files (where they will be served from)
        STATIC_URL = '/static/'
        ```

    - `whitenoise` 미들웨어 추가

        ```py
        MIDDLEWARE = [
            'django.middleware.security.SecurityMiddleware',
            'whitenoise.middleware.WhiteNoiseMiddleware',       # 추가!
            'django.contrib.sessions.middleware.SessionMiddleware',
            'django.middleware.common.CommonMiddleware',
            'django.middleware.csrf.CsrfViewMiddleware',
            'django.contrib.auth.middleware.AuthenticationMiddleware',
            'django.contrib.messages.middleware.MessageMiddleware',
            'django.middleware.clickjacking.XFrameOptionsMiddleware',
        ]
        ```

    - `settings.py` 최하단에 다음 내용 추가(static 파일 용량 감소)

        ```py
        # Simplified static file serving.
        # https://warehouse.python.org/project/whitenoise/
        STATICFILES_STORAGE = 'whitenoise.storage.CompressedManifestStaticFilesStorage'
        ```
    
2. 기타 `settings.py` 내용 설정

    - `ALLOWED_HOSTS`

        ```py
        ALLOWED_HOSTS = ['<your app URL without the https:// prefix>.herokuapp.com','127.0.0.1']
        # For example: 
        # ALLOWED_HOSTS = ['instagram.herokuapp.com','127.0.0.1']
        ```

### V. `git`을 통한 heroku 배포

1. `.gitignore` 추가

    - [Django용 gitignore 템플릿](http://gitignore.io/api/django)
    - `venv/` 추가

        ```gitignore
        ...

        # venv
        venv/
        ```
2. `git`으로 현재 상태 versioning

    ```shell
    $ git init
    $ git add .
    $ git commit -m 'Init project'
    ```

3. heroku 앱 생성 및 앱으로 코드 push

    1. heroku 앱 생성

        ```shell
        $ heroku create [원하는 URL 이름]
        ```
    
    2. `remote` 확인: `heroku`라는 이름의 remote 확인 가능

        ```shell
        $ git remote -v
        heroku  https://git.heroku.com/instagram.git (fetch)
        heroku  https://git.heroku.com/instagram.git (push)
        ```

    3. heroku 리모트 서버로 `git push`

        ```shell
        $ git push heroku master
        ```

### VI. 추가 설정

1. 배포용 설정 사항 확인
    - 다양한 배포 관련 설정 상태 확인
        ```shell
        $ python manage.py check --deploy
        ```
2. `SECRET_KEY`를 환경변수로 관리하기

    - 참조 문서
        - [MDN Django 튜토리얼 파트 11: Django 웹사이트 공개하기](https://developer.mozilla.org/ko/docs/Learn/Server-side/Django/Deployment#Getting_your_website_ready_to_publish)

        - [Django - settings.py 의 SECRET_KEY 변경 및 분리하기](https://wayhome25.github.io/django/2017/07/11/django-settings-secret-key/)

        - [9 Straightforward steps for deploying django app with heroku](https://medium.com/agatha-codes/9-straightforward-steps-for-deploying-your-django-app-with-heroku-82b952652fb4)

        - [Collectstatic error while deploying Django app to Heroku](https://stackoverflow.com/questions/36665889/collectstatic-error-while-deploying-django-app-to-heroku)

    - `settings.py` 설정

        ```py
        # settings.py
        ...

        # SECURITY WARNING: keep the secret key used in production secret!
        # SECRET_KEY = '7x=%38t5_4=uzf0&!#=a^te=&mold=wfbg2z_5z5-$s@g-lxr4'
        SECRET_KEY = os.environ.get('DJANGO_SECRET_KEY', '7x=%38t5_4=uzf0&!#=a^te=&mold=wfbg2z_5z5-$s@g-lxr4')

        # SECURITY WARNING: don't run with debug turned on in production!
        # DEBUG = True
        DEBUG = bool(os.environ.get('DJANGO_DEBUG', True))

        ```

    - Windows CMD에서 `heroku config` 설정

        > Git Bash에서 작업할 경우 `<SECRET_KEY>`의 문자열 부분을 명령어로 착각해 에러 발생

        ```shell
        $ heroku config:set DJANGO_SECRET_KEY="p^zc&@mmo8p6s%g^wd^4(ej4+a^7y5s^j3+ign9mg^io=1-qz&"
        ```

3. local, production 설정 분리