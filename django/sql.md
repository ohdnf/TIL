# SQL과 django ORM

## 참고문서

[Making queries | Django documentation | Django](https://docs.djangoproject.com/en/2.2/topics/db/queries/#making-queries)

[QuerySet API reference | Django documentation | Django](https://docs.djangoproject.com/en/2.2/ref/models/querysets/#queryset-api-reference)

[Aggregation | Django documentation | Django](https://docs.djangoproject.com/en/2.2/topics/db/aggregation/#aggregation)

## **기본 준비 사항**

> Gitlab에서 프로젝트를 다운받으면 아래의 내용이 이미 반영되어 있습니다.

- django app
    - `django_extensions` 설치
    - `users` app 생성
    - csv 파일에 맞춰 `models.py` 작성 및 migrate

            $ python manage.py sqlmigrate users 0001

- `db.sqlite3` 활용 및 데이터 반영
    - `sqlite3` 실행

            $ ls
            db.sqlite3 manage.py ...
            $ sqlite3 db.sqlite3

    - csv 파일 data 로드

            sqlite > .tables
            auth_group                  django_admin_log
            auth_group_permissions      django_content_type
            auth_permission             django_migrations
            auth_user                   django_session
            auth_user_groups            auth_user_user_permissions
            users_user
            sqlite > .mode csv
            sqlite > .import users.csv users_user
            sqlite > SELECT COUNT(*) FROM users_user;
            1000

- 확인
    - sqlite3에서 스키마 확인

            sqlite > .schema users_user

## **문제**

> 아래의 문제들을 sql문과 대응되는 orm을 작성 하세요.

### Table 생성

- django

    ```python
    # django
    class User(models.Model):
        first_name = models.CharField(max_length=10)
        last_name = models.CharField(max_length=10)
        age = models.IntegerField()
        country = models.CharField(max_length=10)
        phone = models.CharField(max_length=15)
        balance = models.IntegerField()
    ```

- SQL
    - sql.sqlite3에 동일하게 테이블 생성

        ```sql
        --sql
        CREATE TABLE users (
           id INTEGER PRIMARY KEY AUTOINCREMENT,
           first_name TEXT NOT NULL,
           last_name TEXT NOT NULL,
           age INTEGER NOT NULL,
           country TEXT NOT NULL,
           phone TEXT,
           balance INTEGER
        );
        ```

### 기본 CRUD 로직

1. 모든 user 레코드 조회

   ```python
   # orm
   User.objects.all()
   ```

   ```sql
   -- sql
   SELECT * FROM users;
   ```

2. user 레코드 생성

   ```python
   # orm
   User.objects.create(first_name='Tony', last_name='Stark', age=30, country='USA', phone='1-23-456-789', balance=9876543210)
   ```

   ```sql
   -- sql
   INSERT INTO users VALUES('Tony', 'Stark', 30, 'USA', '1-23-456-789', 9876543210);
   ```

   * 하나의 레코드를 빼고 작성 후 `NOT NULL` constraint 오류를 orm과 sql에서 모두 확인 해보세요.

3. 해당 user 레코드 조회

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

4. 해당 user 레코드 수정

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

5. 해당 user 레코드 삭제

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

### 조건에 따른 쿼리문

1. 전체 인원 수

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

2. 나이가 30인 사람의 이름

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

3. 나이가 30살 이상인 사람의 인원 수

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

4. 나이가 30이면서 성이 김씨인 사람의 인원 수

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

5. 지역번호가 02인 사람의 인원 수

   > https://docs.djangoproject.com/en/2.2/topics/db/queries/#escaping-percent-signs-and-underscores-in-like-statements

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

6. 거주 지역이 강원도이면서 성이 황씨인 사람의 이름

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```



### 정렬 및 LIMIT, OFFSET

1. 나이가 많은 사람 10명

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

2. 잔액이 적은 사람 10명

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

3. 성, 이름 내림차순 순으로 5번째 있는 사람

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```



### 표현식

> 표현식을 위해서는 [aggregate]([https://docs.djangoproject.com/en/2.2/topics/db/aggregation/](https://docs.djangoproject.com/en/2.2/topics/db/aggregation/)) 를 알아야한다.

1. 전체 평균 나이

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

2. 김씨의 평균 나이

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

3. 계좌 잔액 중 가장 높은 값

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

4. 계좌 잔액 총액

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```



### Group by

> annotate는 개별 item에 추가 필드를 구성한다.
> 추후 1:N 관계에서 활용된다.

1. 지역별 인원 수

   ```python
   # orm
   ```

   ```sql
   -- sql
   ```

