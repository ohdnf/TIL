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

> 테이블 생성 예시

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

## 테이블 삭제