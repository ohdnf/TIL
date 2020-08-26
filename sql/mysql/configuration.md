# MySQL 데이터베이스 초기 설정



## DBMS 접속

```mysql
mysql -u root -p
```



> 접속이 안 된다면 JS mode가 아닌 SQL mode로 되어 있는지 확인하자

```mysql
MySQL JS> \sql
MySQL SQL> \connect root@localhost
```



## DB 생성 및 사용

```mysql
# DB 생성
CREATE DATABASE miniter;
# 생성한 DB 사용
USE miniter;
```



## SQL 활용한 테이블 생성 및 확인

```mysql
# users 테이블 생성
CREATE TABLE users(
id INT NOT NULL auto_increment,
name VARCHAR(255) NOT NULL,
email VARCHAR(255) NOT NULL,
hashed_password VARCHAR(255) NOT NULL,
profile VARCHAR(2000) NOT NULL,
created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
updated_at TIMESTAMP null DEFAULT NULL ON UPDATE CURRENT_TIMESTAMP,
PRIMARY KEY (id),
unique key email (email)
);

# 생성된 테이블 확인
SHOW TABLES;
# 결과
+-------------------+
| Tables_in_miniter |
+-------------------+
| users             |
+-------------------+

# 상세 정보 확인
EXPLAIN users;
# 결과
+-----------------+---------------+------+-----+-------------------+-----------------------------+
| Field           | Type          | Null | Key | Default           | Extra                       |
+-----------------+---------------+------+-----+-------------------+-----------------------------+
| id              | int           | NO   | PRI | NULL              | auto_increment              |
| name            | varchar(255)  | NO   |     | NULL              |                             |
| email           | varchar(255)  | NO   | UNI | NULL              |                             |
| hashed_password | varchar(255)  | NO   |     | NULL              |                             |
| profile         | varchar(2000) | NO   |     | NULL              |                             |
| created_at      | timestamp     | NO   |     | CURRENT_TIMESTAMP | DEFAULT_GENERATED           |
| updated_at      | timestamp     | YES  |     | NULL              | on update CURRENT_TIMESTAMP |
+-----------------+---------------+------+-----+-------------------+-----------------------------+
```



