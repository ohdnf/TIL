sqlite3

```bash
$ pip install sqlite3
```

```bash
$ sqlite3 exercise.sqlite3
```

## DDL

### 테이블 생성

```sql
sqlite> CREATE TABLE flights (
   ...> id INTEGER PRIMARY KEY AUTOINCREMENT,
   ...> flight_num TEXT NOT NULL,
   ...> departure TEXT NOT NULL,
   ...> waypoint TEXT NOT NULL,
   ...> arrival TEXT NOT NULL,
   ...> price INTEGER NOT NULL
   ...> );
sqlite> .tables
flights
sqlite>
```

### 테이블 삭제

```sql
sqlite> DROP TABLE flights;
```

## DML: CRUD

### Create

    ```sql
    sqlite3> INSERT INTO flights (id, flight_num, departure, waypoint, arrival, price) VALUES (1, 'RT9122', 'Madrid', 'Beijing', 'Incheon', 200);
    ```

- 컬럼을 모두 입력할 경우 VALUES 앞에 컬럼을 생략할 수 있다.

    ```sql
    sqlite3> INSERT INTO flights VALUES (2, 'XZ0352', 'LA', 'Moscow', 'Incheon', 800);
    ```

- `id` 컬럼은 자동으로 입력하고 싶을 땐 다음과 같이 입력한다.

    ```sql
    INSERT INTO flights (flight_num, departure, waypoint, arrival, price) VALUES ('SQ0972', 'London', 'Beijing', 'Sydney', 500);
    ```

### Read

```sql
sqlite3> SELECT * FROM flights;
id | flight_num | departure | waypoint | arrival | price
1 | RT9122 | Madrid | Beijing | Incheon | 200
```

> 깔끔하게 보는 옵션

```sql
sqlite3> .headers on
sqlite3> .mode column
sqlite> SELECT * FROM flights;
id          flight_num  departure   waypoint    arrival     price     
----------  ----------  ----------  ----------  ----------  ----------
1           RT9122      Madrid      Beijing     Incheon     200       
2           XZ0352      LA          Moscow      Incheon     800       
3           SQ0972      London      Beijing     Sydney      500
```

- 특정 컬럼만 조회

```sql
sqlite3> SELECT waypoint FROM flights;
waypoint  
----------
Beijing   
Moscow    
Beijing 
```

- 중복 없이 조회

```sql
sqlite3> SELECT DISTINCT waypoint FROM flights;
waypoint  
----------
Beijing   
Moscow
```

- 조건을 넣어 조회

```sql
sqlite> SELECT id, flight_num FROM flights WHERE price < 600;
id          flight_num
----------  ----------
1           RT9122    
3           SQ0972
```

```sql
sqlite> SELECT departure FROM flights WHERE arrival='Incheon' AND price >= 500;                                                               
departure 
----------
LA 
```

- REGEX 맛보기

> 0으로 시작하고 2로 끝나는 항공편 찾기

```sql
sqlite> SELECT id, flight_num FROM flights WHERE waypoint='Beijing' AND flight_num LIKE '__0__2';                                             
id          flight_num
----------  ----------
3           SQ0972
```

```sql
sqlite> SELECT id, flight_num FROM flights WHERE waypoint='Beijing' AND flight_num LIKE '%0__2';
id          flight_num
----------  ----------
3           SQ0972
```

### Update

- 조건부 수정

```sql
sqlite> UPDATE flights SET waypoint = 'Tokyo' WHERE flight_num = 'SQ0972';
sqlite> SELECT * FROM flights;
id          flight_num  departure   waypoint    arrival     price     
----------  ----------  ----------  ----------  ----------  ----------
1           RT9122      Madrid      Beijing     Incheon     200       
2           XZ0352      LA          Moscow      Incheon     800       
3           SQ0972      London      Tokyo       Sydney      500 
```

### Delete

- 조건부 삭제

```sql
sqlite> DELETE FROM flights WHERE flight_num = 'RT9122';
sqlite> SELECT * FROM flights;
id          flight_num  departure   waypoint    arrival     price     
----------  ----------  ----------  ----------  ----------  ----------
2           XZ0352      LA          Moscow      Incheon     800       
3           SQ0972      London      Tokyo       Sydney      500
```