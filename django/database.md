# Database

## Schema


## Model

| 구분 | 의미 |
| --- | ---- |
| Model | MTV 패턴에서 데이터를 관리 |
| Migration | Model로 정의된 클래스를 데이터베이스 스키마에 반영 |
| ORM(Query methods, QuerySet API) | 데이터베이스를 조작 |

### Model 생성

```py
# models.py
from django.db import models

# models.Model을 반드시 상속받아야 한다.
class Article(models.Model):
    # CharField()는 max_length parameter에 인자를 반드시 넘겨줘야 한다.
    title = models.CharField(max_length=20)
    content = models.TextField()
    # auto_now_add: 인스턴스 생성 시에만 자동으로 현재 시간 저장
    # auto_now: 인스턴스가 save() 될 때마다 자동으로 현재 시간 저장
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    def __repr__(self):
        """
        해당 메소드를 통해 모델 인스턴스가 출력될 때 나올 값을 지정해줄 수 있다.
        """
        return f'{self.title} created at {self.created_at}'
```

## ORM: Object Relational Mapping

[Django ORM Cookbook]()

### makemigrations & migrate

method나 컬럼 옵션들은 schema 반영을 하지 않아도 쓸 수 있다.

### DB 만들기

- `.csv` 파일을 데이터로 불러오기

```shell
# 똑같은 명령어
$ sqlite3 db.sqlite3 # 직접 실행
$ python manage.py dbshell # django가 알아서 dbms 열어줌
```

```sql
-- header 컬럼을 날리고 실행
sqlite> .mode csv
-- .import {파일명} {테이블명}
sqlite> .import users.csv users_user
sqlite> .schema users_user
sqlite> .headers on
```

### DB 초기화하기
> 주의: 모든 데이터가 사라짐!
1. `migrations/` 내 `__init__.py` 파일 제외(삭제하면 패키지로 인식하지 못 함)하고 이력(`0001_initial.py` 등) 삭제
2. `db.sqlite3` 삭제
3. `$ python manage.py makemigrations`
4. `$ python manage.py migrate [app_label] [migration_name]`

### DB 확인

`$ python manage.py showmigrations`

## SQL

`$ python manage.py sqlmigrate app_label migration_name`

예) `$python manage.py sqlmigrate article 0001`

migrate할 때 실제 대응되는 SQL문을 보여준다.

### 조회: `SELECT`

#### `get`

- 오직 하나 반환 or 에러

#### `filter`

- `QuerySet` 반환

- AND
    ```py
    Posts.object.filter(condition_1, condition_2)
    ```

- OR
    ```py
    Posts.object.filter(Q(condition_1) | Q(condition_2))
    ```

### 대소관계

### 표현식 사용: Aggregation


### python shell_plus 에서 sql문 바로 보기

```shell
$ python manage.py shell_plus --print-sql
```

## Relation

### 1:N

[Django Documentation](https://docs.djangoproject.com/en/3.0/topics/db/queries/#one-to-many-relationships)

> 1 has many N
> N belongs to 1

```py
# models.py
import django.db import models

class Reporter(models.Model):
    username = models.CharField(max_length=20)

class Article(models.Model):
    title = models.CharField(max_length=100)
    conent = models.TextField()
    reporter = models.ForeignKey(Reporter, on_delete=models.CASCADE)
```

## Naming Convention

```py
reporter.article_set.all() # 자동으로 설정

# models.py
class Reporter(models.Model):
    ...
    article = models.ForeignKey(
        Article, on_delete=models.CASCADE, 
        related_name='reporter')
article.reporter # related_name 옵션으로 직접 설정
```

### 1:N

- 정의: 모델 단수형(`.user`)
- 역참조: 모델_set(`.article_set`)

### M:N

- 정의 및 역참조: 모델 복수형(`.like_users`, `like_articles`)
