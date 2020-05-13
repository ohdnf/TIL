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

    - aws
    - Azure
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

모든 작품은 세상에 공개(Ship)될 때, 비로소 완성되며 동시에 생(Life)을 시작한다.
완성은 시작보다 어려운 일이며, 공개는 겁나는 일이다.

100개의 걸출한 미완의 습작보다 1개의 완성된 졸작이 훨씬 더 큰 의미를 갖는다.



## Heroku

[참고 링크](https://developer.mozilla.org/en-US/docs/Learn/Server-side/Django/Deployment)

### 1. `Procfile`

```procfile
web: gunicorn locallibrary.wsgi --log-file -
```

### 2. `runtime.txt`

```txt
python-3.7.6
```

### 3. `requirements.txt`

```shell
$ pip install gunicorn whitenoise dj-database-url
$ pip freeze > requirements.txt
```

```txt
dj-database-url==0.5.0
Django==2.1.15
gunicorn==20.0.4
pytz==2020.1
whitenoise==5.0.1
psycopg2==2.7.7
```

> `psycopg2==2.7.7`는 직접 입력해준다.