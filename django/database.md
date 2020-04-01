# Database

## Schema


## Model

| 구분 | 의미 |
| --- | ---- |
| Model | MTV 패턴에서 데이터를 관리 |
| Migration | Model로 정의된 데이터베이스 스키마를 반영 |
| ORM | Python 객체 조작을 통해 데이터베이스를 조작 |

## ORM: Object Relational Mapping


## DB 초기화하기
1. `migrations/` 내 `__init__.py` 파일 제외하고 이력(`0001_initial.py` 등) 삭제
2. `db.sqlite3` 삭제
3. `$ python manage.py makemigrations`
4. `$ python manage.py migrate`