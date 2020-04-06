# Database

## Schema


## Model

| 구분 | 의미 |
| --- | ---- |
| Model | MTV 패턴에서 데이터를 관리 |
| Migration | Model로 정의된 데이터베이스 스키마를 반영 |
| ORM | Python 객체 조작을 통해 데이터베이스를 조작 |

### Model 생성



## ORM: Object Relational Mapping

### DB 초기화하기
1. `migrations/` 내 `__init__.py` 파일 제외하고 이력(`0001_initial.py` 등) 삭제
2. `db.sqlite3` 삭제
3. `$ python manage.py makemigrations`
4. `$ python manage.py migrate [app_label] [migration_name]`

### DB 확인
`$ python manage.py showmigrations`

### SQL

`$ python manage.py sqlmigrate app_label migration_name`

예) `$python manage.py sqlmigrate article 0001`

migrate할 때 실제 대응되는 SQL문을 보여준다.