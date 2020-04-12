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

### DB 초기화하기
> 주의: 모든 데이터가 사라짐!
1. `migrations/` 내 `__init__.py` 파일 제외(삭제하면 패키지로 인식하지 못 함)하고 이력(`0001_initial.py` 등) 삭제
2. `db.sqlite3` 삭제
3. `$ python manage.py makemigrations`
4. `$ python manage.py migrate [app_label] [migration_name]`

### DB 확인
`$ python manage.py showmigrations`

### SQL

`$ python manage.py sqlmigrate app_label migration_name`

예) `$python manage.py sqlmigrate article 0001`

migrate할 때 실제 대응되는 SQL문을 보여준다.
