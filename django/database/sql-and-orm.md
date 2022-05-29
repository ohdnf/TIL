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
          CREATE TABLE IF NOT EXISTS "users_user" ("id" integer NOT NULL PRIMARY KEY AUTOINCREMENT, "first_name" varchar(10) NOT NULL, "last_name" varchar(10) NOT NULL, "age" integer NOT NULL, "country" varchar(10) NOT NULL, "phone" varchar(15) NOT NULL, "balance" integer NOT NULL);

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

    )
    ```

### 기본 CRUD 로직

1. 모든 user 레코드 조회

   ```python
   # orm
   users = User.objects.all()
   ```

   ```sql
   -- sql
   SELECT * FROM users_user;
   ```

2. user 레코드 생성

   ```python
   # orm
   new = User(last_name='김', age=18, country='서울특별시', phone='010-9876-5432', balance=10000000)
   new.save()
   ```

   ```sql
   -- sql
   INSERT INTO "users_user" ("first_name", "last_name", "age", "country", "phone", "balance") VALUES ('', '김', 18, '서울특별시', '010-9876-5432', 10000000);
   ```

   - 하나의 레코드를 빼고 작성 후 `NOT NULL constraint` 오류를 orm과 sql에서 모두 확인 해보세요.

   ```python
   new = User.objects.create(first_name='Bruce', last_name='Springsteen')
   ```

   ```sql
   INSERT INTO "users_user" ("first_name", "last_name", "age", "country", "phone", "balance") VALUES ('Bruce', 'Springsteen', NULL, '', '', NULL);
   ```

   ```
   IntegrityError: NOT NULL constraint failed: users_user.age
   ```

3. 해당 user 레코드 조회

   ```python
   # orm
   # id로 조회하면 단일 레코드로 반환
   kim = User.objects.get(pk=102)
   # id를 모른다면 filter 조건으로 조회 ==> QuerySet 반환
   kim = User.objects.filter(last_name='김', age=18, country='서울특별시')[0]
   ```

   ```sql
   -- sql
   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" WHERE "users_user"."id" = 102;

   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" WHERE ("users_user"."age" = 18 AND "users_user"."country" = '서울특별시' AND "users_user"."last_name" = '김')  LIMIT 1;
   ```

4. 해당 user 레코드 수정

   ```python
   # orm
   kim.first_name = '선달'
   kim.save()
   ```

   ```sql
   -- sql
   UPDATE "users_user" SET "first_name" = '선달', "last_name" = '김', "age" = 18, "country" = '서울특별시', "phone" = '010-9876-5432', "balance" = 10000000 WHERE "users_user"."id" = 102;
   ```

5. 해당 user 레코드 삭제

   ```python
   # orm
   hong = User.objects.get(id=101)
   hong.delete()
   ```

   ```sql
   -- sql
   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" WHERE "users_user"."id" = 101;

   DELETE FROM "users_user" WHERE "users_user"."id" IN (101);
   ```

### 조건에 따른 쿼리문

1. 전체 인원 수

   ```python
   # orm
   User.objects.all().count()
   ```

   ```sql
   -- sql
   SELECT COUNT(*) AS "__count" FROM "users_user";
   ```

2. 나이가 30인 사람의 이름

   ```python
   # orm
   dirty_sexy = User.objects.filter(age=30)

   for user in dirty_sexy:
      print(user.last_name, user.first_name)
   ```

   ```sql
   -- sql
   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" WHERE "users_user"."age" = 30;
   ```

3. 나이가 30살 이상인 사람의 인원 수

   ```python
   # orm
   User.objects.filter(age__gte=30).count()
   ```

   ```sql
   -- sql
   SELECT COUNT(*) AS "__count" FROM "users_user" WHERE "users_user"."age" >= 30;
   ```

4. 나이가 30이면서 성이 김씨인 사람의 인원 수

   ```python
   # orm
   User.objects.filter(age=30, last_name='김').count()
   ```

   ```sql
   -- sql
   SELECT COUNT(*) AS "__count" FROM "users_user" WHERE ("users_user"."age" = 30 AND "users_user"."last_name" = '김');
   ```

5. 지역번호가 02인 사람의 인원 수

   > https://docs.djangoproject.com/en/2.2/topics/db/queries/#escaping-percent-signs-and-underscores-in-like-statements

   ```python
   # orm
   User.objects.filter(phone__startswith='02').count()

   ## 또는 Q를 쓸 수도 있다(쿼리 동일)
   from django.db.models import Q
   User.objects.filter(Q(age=30) & Q(last_name='김')).count()
   ```

   ```sql
   -- sql
   SELECT COUNT(*) AS "__count" FROM "users_user" WHERE "users_user"."phone" LIKE '02%' ESCAPE '\';
   ```

6. 거주 지역이 강원도이면서 성이 황씨인 사람의 이름

   ```python
   # orm
   User.objects.get(country='강원도', last_name='황').first_name
   ```

   ```sql
   -- sql
   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" WHERE ("users_user"."country" = '강원도' AND "users_user"."last_name" = '황');
   ```

### 정렬 및 LIMIT, OFFSET

1. 나이가 많은 사람 10명

   ```python
   # orm
   User.objects.order_by('-age')[:10]
   ```

   ```sql
   -- sql
   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" ORDER BY "users_user"."age" DESC  LIMIT 10;
   ```

2. 잔액이 적은 사람 10명

   ```python
   # orm
   User.objects.order_by('balance')[:10]
   ```

   ```sql
   -- sql
   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" ORDER BY "users_user"."balance" ASC  LIMIT 10;
   ```

3. 성, 이름 내림차순 순으로 5번째 있는 사람

   ```python
   # orm
   User.objects.order_by('-last_name', '-first_name')[4]
   ```

   ```sql
   -- sql
   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" ORDER BY "users_user"."last_name" DESC, "users_user"."first_name" DESC  LIMIT 1 OFFSET 4;
   ```

### 표현식

> 표현식을 위해서는 [`aggregate`](https://docs.djangoproject.com/en/2.2/topics/db/aggregation/)를 알아야한다.

1. 전체 평균 나이

   ```python
   # orm
   User.objects.all().aggregate(Avg('age'))
   ```

   ```sql
   -- sql
   SELECT AVG("users_user"."age") AS "age__avg" FROM "users_user";
   ```

2. 김씨의 평균 나이

   ```python
   # orm
   User.objects.filter(last_name='김').aggregate(Avg('age'))
   ```

   ```sql
   -- sql
   SELECT AVG("users_user"."age") AS "age__avg" FROM "users_user" WHERE "users_user"."last_name" = '김';
   ```

3. 계좌 잔액 중 가장 높은 값

   ```python
   # orm
   User.objects.all().aggregate(Max('balance'))

   # User.objects.order_by('-balance')[0].balance
   ```

   ```sql
   -- sql
   SELECT MAX("users_user"."balance") AS "balance__max" FROM "users_user";
   Execution time: 0.000000s [Database: default]

   SELECT "users_user"."id", "users_user"."first_name", "users_user"."last_name", "users_user"."age", "users_user"."country", "users_user"."phone", "users_user"."balance" FROM "users_user" ORDER BY "users_user"."balance" DESC  LIMIT 1;
   Execution time: 0.001000s [Database: default]
   ```

4. 계좌 잔액 총액

   ```python
   # orm
   User.objects.all().aggregate(Sum('balance'))
   ```

   ```sql
   -- sql
   SELECT SUM("users_user"."balance") AS "balance__sum" FROM "users_user";
   ```

### Group by

> [`annotate`](https://docs.djangoproject.com/en/3.0/ref/models/querysets/#annotate)는 개별 item에 추가 필드를 구성한다.
> 추후 1:N 관계에서 활용된다.

1. 지역별 인원 수

   ```python
   # orm
   User.objects.values('country').annotate(Count('country'))
   ```

   ```sql
   -- sql
   SELECT "users_user"."country", COUNT("users_user"."country") AS "country__count" FROM "users_user" GROUP BY "users_user"."country";
   ```
