[TOC]

# 누구나 쉽게 SQL 요약

## SQL의 종류

| 분류 | 개념 | 예시 |
| --- | --- | --- |
| DDL | 데이터 정의: RDBMS(테이블, 스키마)를 정의하기 위한 명령어 | CREATE/DROP/ALTER |
| DML | 데이터 CRUD | INSERT/UPDATE/DELETE/SELECT |
| DCL | DB 사용자 권한 제어 등 | GRANT/REVOKE/COMMIT/ROLLBACK |


### DDL(Data Definition Language)

- `CREATE`: 객체 생성
- `DROP`: 객체 삭제
- `ALTER`: 객체 변경
- `TRUNCATE TABLE`: 테이블의 모든 데이터 삭제
- `RENAME`: 객체 이름 변경

### DML(Data Manipulation Language)

- `SELECT`: 테이블이나 뷰에서 데이터 조회
- `INSERT`: 데이터 입력
- `UPDATE`: 기존 데이터 수정
- `DELETE`: 테이블의 데이터 삭제
- `MERGE`: 조건에 따라 `INSERT`와 `UPDATE` 수행

> `TRUNCATE TABLE`은 복구 불가. `DELETE`는 조건부 삭제 또는 삭제 이전 이점으로 복원 가능. 데이터 삭제할 때는 `DELETE` 사용

### TCL(Transaction Control Language)

- `COMMIT`: DML로 변경된 데이터를 DB에 적용
- `ROLLBACK`: DML로 변경된 데이터를 변경 이전 상태로 되돌림

### DCL(Data Control Language)

- `GRANT`: 객체에 대한 권한 할당
- `REVOKE`: 객체에 할당한 권한 회수
 
## 테이블 생성

### 테이블 생성 구문

```sql
CREATE TABLE table_name (
    column_name1 datatype [NOT] NULL,
    column_name2 datatype [NOT] NULL,
    ...
    PRIMARY KEY ( column_list )
);
```

- `table_name`: 테이블 이름
- `column_name`: 컬럼 이름
- `datatype`: 컬럼의 데이터 유형
- `[NOT] NULL`: 해당 컬럼의 NULL 허용 여부(default: NULL 허용)

### 데이터 유형

| 데이터 유형 | 데이터형 | 설명 |
| ---------- | ------- | ---- |
| 문자형 | CHAR(n) | 고정 길이 문자, 최대 2000byte |
|  | VARCHAR2(n) | 가변 길이 문자, 최대 4000byte |
| 숫자형 | NUMBER[(p, [s])] | p(1~38, default값은 38)와 s(-84~127, default값은 0)는 십진수 기준, 최대 22byte |
| 날짜형 | DATE | BC 4712년 1월 1일부터 9999년 12월 31일까지 년, 월, 일, 시, 분, 초까지 입력 가능 |

- 테이블 생성 예시

```sql

CREATE TABLE emp03
(
    emp_id      NUMBER          NOT NULL,
    emp_name    VARCHAR2(100)   NOT NULL,
    gender      VARCHAR2(10)        NULL,
    age         NUMBER              NULL,
    hire_date   DATE                NULL,
    etc         VARCHAR2(300)       NULL,
    PRIMARY KEY ( emp_id )
);
```

## 데이터 입력

```sql
INSERT INTO table_name ( column1, column2, column3, ... )
VALUES ( value1, value2, value3, ... );
```

> 지하철 통계 데이터 입력 예시

```sql
CREATE TABLE subway_statistics (
     seq_id            NUMBER        NOT NULL,
     station_name      VARCHAR2(100)     NULL,
     boarding_date     DATE              NULL,
     gubun             VARCHAR2(10)      NULL,
     boarding_time     NUMBER            NULL,
     passenger_number  NUMBER            NULL,
     PRIMARY KEY ( seq_id )
);

INSERT INTO subway_statistics values(1,'서울역(150)','2017-04-01','승차',7,654);
...
```

- `VARCHAR2`형은 값을 따옴표(`'`)로 묶어야 함
- `DATE`형은 따옴표로 묶어서 넣어도 오라클에서 날짜로 변환해서 넣어줌
- *컬럼명을 명시하지 않을 경우 모든 컬럼에 데이터를 넣는다*는 것을 의미
- **일반적으로 컬럼명을 명시**하는 것이 가독성 측면에서 좋고, 컬럼 순서와 값을 잘못 매핑하는 실수를 방지

### `COMMIT`

- 데이터 변경 후에는 DB에 변경사항을 적용하는 트랜잭션 처리를 해야 함

```SQL
COMMIT;
```

- 반대로 데이터 입력이나 변경을 취소하고 싶다면 `ROLLBACK` 문장 실행

## 데이터 조회

```SQL
SELECT column1, column2, ...
FROM table_name
WHERE condition
ORDER BY order_cond;
```

### `SELECT`

- 테이블에서 선택할 컬럼이나 표현식을 명시
- `*`를 명시하면 테이블에 있는 전체 컬럼을 선택
- 표현식 사용 가능

### `FROM`

- 조회하고자 하는 테이블 명시

### `WHERE`

- 특정 조건에 맞는 데이터를 가져오고자 할 때 해당 조건 기술

### `LIKE`

> station_name 컬럼에서 `서울`로 시작된 모든 레코드를 조회할 때

```sql
-- 서울역 조회
WHERE station_name LIKE '서울%'
```

### `IN`

> 선릉역에서 7시와 9시 승하차 건을 조회

```sql
SELECT *
FROM subway_statistics
WHERE station_name LIKE '선릉%'
AND boarding_time IN (7, 9);
```

### `ORDER BY`

- 조회한 데이터를 정렬해서 보여주는 역할
- 정렬하고자 하는 컬럼 끝에 `ASC`, 또는 `DESC` 명시

```sql
-- col1 컬럼 오름차순 후, col2 컬럼 오름차순 정렬
ORDER BY col1, col2

-- col1 컬럼 내림차순 후, col2 컬럼 오름차순 정렬
ORDER BY col1 DESC, col2

-- col2 컬럼 오름차순 후, col1 컬럼 내림차순 정렬
ORDER BY col2, col1 DESC

-- SELECT 절에 명시한 첫 번째와 두 번째 컬럼을 순서대로 오름차순 정렬
ORDER BY 1, 2
```

## SQL 연산자, 함수

### 기본 연산자

```sql
-- 문자열 결합 연산자
SELECT 'A' || 'B' FROM dual;
```

> `dual` 테이블은 오라클에서만 사용할 수 있는 임시 테이블

### 숫자형 함수

#### `ROUND(n, i)`

`n`의 소수점 기준 (`i`+1)번째에서 반올림한 값을 반환, 정수로 만들 시 `i`는 0(소수점 첫째 자리 기준 반올림)

```sql
SELECT ROUND(3.141592, 2) FROM dual;
-- 결과: 3.14
```

`i`가 음수일 경우 소수점 기준 왼쪽 자리인 정수 부분의 `i`번째 자리 수를 반올림한 결과를 반환

```sql
SELECT ROUND( 565.545, -1 ) first,
       ROUND( 565.545, -2 ) second,
       ROUND( 565.545, -3 ) third
    FROM dual;
-- FIRST: 570 SECOND: 600 THIRD: 1000
```

### 문자형 함수

#### `SUBSTR(char, n1, n2)`

- `char`에서 `n1` 위치에서 시작해 `n2` 길이만큼을 잘라낸 결과를 반환
- `n1`을 0으로 명시하면 1이 적용
- `n1`이 음수이면 `char` 오른쪽 끝에서부터 거꾸로 세어 시작
- `n2`를 생략하면 `n1`부터 끝까지 반환
- `n2` 값을 1 미만으로 지정하면 `NULL`을 반환

```sql
SELECT SUBSTR('ABCDEFG', 1, 3) first,   -- ABC
       SUBSTR('ABCDEFG', 0, 3) second,  -- ABC
       SUBSTR('ABCDEFG', 1) third,      -- ABCDEFG
       SUBSTR('ABCDEFG', -2) fourth,    -- FG
       SUBSTR('ABCDEFG', -2, 1) fifth,  -- F
       SUBSTR('ABCDEFG', 2, -1) sixth,  -- NULL
```

#### `INSTR(char1, char2, n1, n2)`

- `char1`에서 `char2` 문자를 찾아 그 시작 위치를 반환
- `n1`은 `char1`에서 몇 번째 문자부터 찾을 것인지를 나타내는 위치이며 생략 시 1이 적용됨
- `n2`는 `char1`에서 `char2` 문자를 찾을 때 일치하는 문자의 몇 번째 위치를 반환할지를 나타냄. 생략 시 1이 적용됨

```sql
SELECT INSTR('ABABAB', 'A') first,          -- 1
       INSTR('ABABAB', 'A', 2) second,      -- 3
       INSTR('ABABAB', 'A', 2, 1) third,    -- 3
       INSTR('ABABAB', 'A', 2, 2) fourth,   -- 5
    FROM dual;
```

### 날짜형 함수

데이터형이 `DATE`나 `TIMESTAMP`인 데이터 대상으로 연산 수행

```sql
-- SYSDATE: 현재 시스템 날짜
SELECT SYSDATE FROM dual;
-- 2020-07-28

-- 기준 날짜에 n개월 더한 날짜 반환
SELECT ADD_MONTHS(SYSDATE, 1) FROM dual;
-- 2020-08-28

-- 두 날짜 사이 개월 수 반환
SELECT MONTHS_BETWEEN(SYSDATE+31, SYSDATE) FROM dual;
-- 1

-- 날짜가 속한 월의 마지막 일자 반환
SELECT LAST_DAY(SYSDATE) FROM dual;
-- 2020-07-31

-- 날짜를 기준으로 돌아올 요일의 날짜 반환
SELECT NEXT_DAY(SYSDATE, '월요일') FROM dual;
-- 2020-08-03
```

### NULL 관련 함수

```sql
-- NVL(expr1, expr2): expr1이 NULL인 경우 expr2를 반환
SELECT NVL(NULL, 'N/A') FROM dual;
-- N/A

-- NVL2(expr1, expr2, expr3): expr1이 NULL인 경우 expr3, 아닌 경우 expr2 반환
SELECT NVL2(1, 2, 3) FROM dual;
-- 2

-- COALESCE(expr1, expr2, ...): 매개변수 중 첫 번째로 NULL이 아닌 값 반환
SELECT COALESCE(NULL, NULL, 5, 4, NULL) FROM dual;
-- 5

-- NULLIF(expr1, expr2): expr1, expr2 비교해 두 값이 같으면 NULL, 같지 않으면 expr1 반환
SELECT NULLIF('null', 'NULL') FROM dual;
-- null
```

> 오라클에서 NULL 값은 `(null)`로 표시

### `CASE`

```sql
SELECT emp_name
      ,age
      ,CASE WHEN age BETWEEN 0  AND 19 THEN '10대'
            WHEN age BETWEEN 20 AND 29 THEN '20대'
            WHEN age BETWEEN 30 AND 39 THEN '30대'
            WHEN age BETWEEN 40 AND 49 THEN '40대'
            WHEN age BETWEEN 50 AND 59 THEN '50대'
            ELSE '60대 이상'
       END ages
FROM EMP03;
```

