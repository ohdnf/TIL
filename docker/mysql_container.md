# Docker로 MySQL Container 만들기



## MySQL Container 생성

```shell
$ docker run -d --name some-mysql -e MYSQL_ROOT_PASSWORD=some-password -p 3306:3306 mysql:5.7
```

- `--name some-mysql`
  - MySQL 컨테이너에 붙일 이름 설정
- `-e MYSQL_ROOT_PASSWORD`
  - MySQL의 ROOT 계정 비밀번호 설정
- `-p 3306:3306`
  - 호스트 머신(예. AWS EC2)의 3306(`:` 앞에 적힌 3306) 포트를 컨테이너의 3306(`:` 뒤에 적힌 3306) 포트와 연결
- `mysql:5.7`
  - 5.7 버전의 MySQL docker image 사용



> **참고**
>
> [mysql docker official images](https://hub.docker.com/_/mysql)
>
> [docker run](https://docs.docker.com/engine/reference/commandline/run/)
>
> [run 명령으로 컨테이너 생성하기](http://pyrasis.com/book/DockerForTheReallyImpatient/Chapter03)



## MySQL Container에 CLI로 접속하기

> mysql를 따로 설치하지 않아도 된다.

```shell
$ docker exec -it mysql_oneta bash
# mysql -u root -p
Enter password:
Welcome to the MySQL monitor.  Commands end with ; or \g.
Your MySQL connection id is 1472
Server version: 5.7.32 MySQL Community Server (GPL)

Copyright (c) 2000, 2020, Oracle and/or its affiliates. All rights reserved.

Oracle is a registered trademark of Oracle Corporation and/or its
affiliates. Other names may be trademarks of their respective
owners.

Type 'help;' or '\h' for help. Type '\c' to clear the current input statement.

mysql>
```



## MySQL에서 새로운 유저 생성하기

```mysql
mysql> use mysql;
Reading table information for completion of table and column names
You can turn off this feature to get a quicker startup with -A

Database changed
mysql> select host, user, authentication_string from user;
+-----------+---------------+-------------------------------------------+
| host      | user          | authentication_string                     |
+-----------+---------------+-------------------------------------------+
| localhost | root          | *95BC39E98A1AA6E0E74F695C2EC74B820DE6E915 |
| localhost | mysql.session | *THISISNOTAVALIDPASSWORDTHATCANBEUSEDHERE |
| localhost | mysql.sys     | *THISISNOTAVALIDPASSWORDTHATCANBEUSEDHERE |
| %         | root          | *95BC39E98A1AA6E0E74F695C2EC74B820DE6E915 |
+-----------+---------------+-------------------------------------------+
4 rows in set (0.00 sec)

mysql> CREATE USER 'hellodev'@'%' IDENTIFIED BY 'typingdev';
Query OK, 0 rows affected (0.00 sec)

mysql> CREATE USER 'hellodev'@'localhost' IDENTIFIED BY 'typingdev';
Query OK, 0 rows affected (0.00 sec)

mysql> select host, user, authentication_string from user;
+-----------+---------------+-------------------------------------------+
| host      | user          | authentication_string                     |
+-----------+---------------+-------------------------------------------+
| localhost | root          | *95BC39E98A1AA6E0E74F695C2EC74B820DE6E915 |
| localhost | mysql.session | *THISISNOTAVALIDPASSWORDTHATCANBEUSEDHERE |
| localhost | mysql.sys     | *THISISNOTAVALIDPASSWORDTHATCANBEUSEDHERE |
| %         | root          | *95BC39E98A1AA6E0E74F695C2EC74B820DE6E915 |
| %         | hellodev      | *95BC39E98A1AA6E0E74F695C2EC74B820DE6E915 |
| localhost | hellodev      | *95BC39E98A1AA6E0E74F695C2EC74B820DE6E915 |
+-----------+---------------+-------------------------------------------+
6 rows in set (0.00 sec)

mysql>
```

- `use mysql;`
  - MySQL에서 유저 관련 데이터베이스인 `mysql`을 사용
- `CREATE USER '유저이름'@'호스트' IDENTIFIED BY '비밀번호';`
  - 호스트
    - `localhost`로 설정하면 해당 계정은 로컬에서만 접속 가능
    - `%`로 설정하면 외부 IP에서 접속 가능
    - `172.168.%`와 같이 설정하면 특정 IP 대역, `172.168.xxx.xxx`에서 접속 가능

> **참고**
>
> [MySQL 계정 생성 관리 및 권한 설정](https://2dubbing.tistory.com/13)
>
> [CREATE USER Statement](https://dev.mysql.com/doc/refman/8.0/en/create-user.html)



## MySQL 스키마 생성 및 권한 부여

> 데이터베이스를 생성하고 생성한 유저에게 권한 부여

```mysql
mysql> CREATE SCHEMA `ONETA` DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_bin ;
mysql> USE ONETA;
mysql> GRANT ALL ON ONETA.* TO hellodev@'%';
Query OK, 0 rows affected (0.00 sec)

mysql> SHOW GRANTS FOR hellodev;
+-----------------------------------------------------+
| Grants for hellodev@%                               |
+-----------------------------------------------------+
| GRANT USAGE ON *.* TO 'hellodev'@'%'                |
| GRANT ALL PRIVILEGES ON `oneta`.* TO 'hellodev'@'%' |
+-----------------------------------------------------+
2 rows in set (0.00 sec)

mysql>
```

- `GRANT ALL ON ONETA.* TO hellodev@'%';`
  - hellodev 유저에게 ONETA 데이터베이스의 모든 테이블(`*`)에 대한 모든 권한 부여
  - 데이터베이스 설정 시 대소문자 구분해야 함

> **참고**
>
> [GRANT Statement](https://dev.mysql.com/doc/refman/8.0/en/grant.html)



## CLI 종료하기

MySQL 접속을 끊고 Bash 창에서 `Ctrl + P`, `Ctrl + Q`를 눌러 컨테이너를 실행 유지하며 CLI를 종료한다.

```shell
mysql> exit;
Bye
# read escape sequence
$
```

