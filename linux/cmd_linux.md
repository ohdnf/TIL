# Terminal Commands(Linux)

| 명령어 | 내용 |
| ------ | ---- |
| `pwd` | 현재 작업 디렉토리 표시 |
| `ls -al` | 현재 작업 디렉토리에 있는 디렉토리 및 파일 표시 |
| `cd dir_path` | 디렉토리 이동(`.` : 현재 디렉토리 / `..` 상위 디렉토리) |
| `mkdir dir_name` | 디렉토리 생성 |
| `rm -r` | 디렉토리 삭제 (폴더 내 파일 Recursive하게 삭제) |
| `touch a.txt` | 파일 `a.txt` 생성 |
| `mv file_path destination` | 파일/폴더를 해당 위치로 이동(이름 변경 가능) |
| `cp file_path destination` | 복사 대상 경로 파일을 복사 위치로 복사(이름 변경 가능) |
| `cat (-n) file_path` | (옵션: `n`행만큼) 파일 내용 출력 |
| `cat file_path > destination` | 기존 파일 내용을 다른 파일로 복사 |
| `echo 'contents' > file_path` | 해당 파일 내용을 `contents`로 덮어쓰기 |
| `echo 'contents' >> file_path` | 해당 파일 맨 뒤에 `contents` 이어쓰기 |
| `start` | `start .`: 현재 디렉토리에서 파일 탐색기 실행 |
| `grep` | 출력된 데이터 중 일치하는 텍스트가 있는 줄들을 보여줌(ex. `pip list | grep django`) |
| `apt-get install 패키지 이름` | 패키지 설치 |
| `apt-get update` | 패키지 업그레이드 가능 여부 체크 |
| `apt-get upgragde` | 업그레이드 가능한 패키지 업그레이드 |
| `chown [OPTIONS] [USER][:GROUP] 파일_또는_폴더` | 파일 또는 폴더의 소유자를 변경 |
| `chmod [OPTIONS] [PERMISSIONS] 파일_또는_폴더` | 파일 또는 폴더의 권한을 변경 |



## linux cmd 단축키 만들기

1. root 디렉토리에 `.bashrc` 파일을 만들어준다.

```shell
$ vi ~/.bashrc
```

```bashrc
alias jn="jupyter notebook"
alias inout="mv ~/Downloads/input.txt ~/Downloads/output.txt ."

# Double command: &&
# cmd1 && cmd2 ==> cmd1 실행 후 cmd2 실행
alias sample="mv ~/Downloads/sample_input.txt ./input.txt && mv ~/Downloads/sample_output.txt ./output.txt"
```

> 띄어쓰기 조심!

- 커스텀 명령어 하나에 두 개 이상의 명령어를 실행하고 싶다면 `;`, `&&`, `&`를 사용한다.
    - `alias custom="cmd1; cmd2"`: 명령어 `cmd1`의 성공여부 상관없이 다음 명령어 `cmd2` 실행
    - `alias custom="cmd1&& cmd2"`: 명령어 `cmd1`가 성공하면 다음 명령어 `cmd2` 실행
    - `alias custom="cmd1& cmd2"`: 명령어 `cmd1`를 백그라운드에서 실행하고 다음 명령어 `cmd2` 실행

`Esc` 눌러서 일반 모드로 나오고 `:wq` 누르고 `Enter` 눌러서 저장 후 종료

2. 이 상태에선 설정이 저장되지 않으므로 **터미널 새로고침**이 필요

```shell
$ source ~/.bashrc
```



## 연속적으로 명령 실행시키기(`;`과 `&`와 `&&`의 차이)

[Opentutorial.org: 리눅스 수업](https://opentutorials.org/module/2538/15818)

### 결론

- `;` 앞의 명령어가 실패해도 다음 명령어 실행
- `&&` 앞의 명령어가 성공하면 다음 명령어 실행
- `&` 앞의 명령어를 백그라운드로 돌리고 동시에 뒤의 명령어를 실행



## 계정 관리하기

> 다중 사용자 운영체제인 Linux에서 사용자 계정을 관리하는 명령어를 알아보자

### 계정 생성

> root 권한이 있는 사용자만 계정을 생성할 수 있습니다.

#### `useradd`

```shell
# -m 옵션을 사용하면 홈 디렉토리도 함께 생성
$ sudo useradd -m user01
[sudo] password for jarvis:
$ sudo passwd user01
Enter new UNIX password:
Retype new UNIX password:
passwd: password updated successfully
# 생성된 사용자 확인
$ tail -n 3 /etc/passwd
jarvis:x:1000:1000:,,,:/home/jarvis:/bin/bash
redis:x:111:115::/var/lib/redis:/usr/sbin/nologin
user01:x:1001:1001::/home/user01:/bin/sh
$ ls /home/
jarvis  user01
```

> ##### `/etc/passwd`에 있는 사용자 파일의 내용
>
> 사용자이름:암호:사용자ID:그룹ID:추가정보:홈디렉토리:쉘

#### `adduser`

```shell
$ sudo adduser user02
[sudo] password for jarvis:
Adding user `user02' ...
Adding new group `user02' (1002) ...
Adding new user `user02' (1002) with group `user02' ...
Creating home directory `/home/user02' ...
Copying files from `/etc/skel' ...
Enter new UNIX password:
Retype new UNIX password:
passwd: password updated successfully
Changing the user information for user02
Enter the new value, or press ENTER for the default
        Full Name []: Second User
        Room Number []:
        Work Phone []:
        Home Phone []:
        Other []:
Is the information correct? [Y/n] Y
$ tail -n 3 /etc/passwd
redis:x:111:115::/var/lib/redis:/usr/sbin/nologin
user01:x:1001:1001::/home/user01:/bin/sh
user02:x:1002:1002:Second User,,,:/home/user02:/bin/bash
$ ls /home/
jarvis  user01  user02
```



### 계정 삭제

#### `userdel`

```shell
$ sudo userdel -r user02
userdel: user02 mail spool (/var/mail/user02) not found
$ tail -n 3 /etc/passwd
jarvis:x:1000:1000:,,,:/home/jarvis:/bin/bash
redis:x:111:115::/var/lib/redis:/usr/sbin/nologin
user01:x:1001:1001::/home/user01:/bin/sh
```



### 계정 변경

#### `su`

```shell
$ su user01
Password:
$ pwd
/home/jarvis
$ exit
$
```



## 파일 접근 권한 변경하기

### `chown`

> "change owner", 즉 파일이나 폴더의 소유자를 변경하는 명령어

```shell
chown [OPTIONS] [USER][:GROUP] 파일_또는_폴더
```

#### 예시

```shell
$ mkdir test
$ touch test/test1.txt test/test2.txt
$ ls -l test/
total 0
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:16 test1.txt
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:16 test2.txt
$ chown user01 test/test1.txt test/test2.txt
$ ls -l test/
total 0
-rw-r--r-- 1 user01 jarvis 0 Nov  5 10:45 test1.txt
-rw-r--r-- 1 user01 jarvis 0 Nov  5 10:45 test2.txt
$ sudo chown :user01 test/test1.txt test/test2.txt
$ ls -l test/
total 0
-rw-r--r-- 1 user01 user01 0 Nov  5 10:45 test1.txt
-rw-r--r-- 1 user01 user01 0 Nov  5 10:45 test2.txt
# 디렉토리 권한 변경
$ sudo chown -R user01:user01 test
$ ls -l
drwxr-xr-x 2 user01 user01 4096 Nov  5 10:45 test
```



### `chmod`

> "change mode", 즉 파일의 권한을 변경하는 명령어

```shell
chmod [OPTIONS] [PERMISSIONS] 파일_또는_폴더
```

#### 권한 설정

```
7: rwx (read, write, and execute)
6: rw- (read and write)
5: r-x (read and execute)
4: r-- (read-only)
3: -wx (write and execute)
2: -w- (write only)
1: --x (execute only)
0: --- (none)
```

#### 예시

```shell
$ mkdir permissions
$ touch permissions/file1 permissions/file2 permissions/file3
$ ls -l permissions/
total 0
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:20 file1
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:20 file2
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:20 file3
$ chmod 777 permissions/file1
$ ls -l permissions/
total 0
-rwxrwxrwx 1 jarvis jarvis 0 Nov  5 10:20 file1
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:20 file2
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:20 file3
$ chmod u=rwx,g=rwx,o=rwx permissions/file2
$ ls -l permissions/
total 0
-rwxrwxrwx 1 jarvis jarvis 0 Nov  5 10:20 file1
-rwxrwxrwx 1 jarvis jarvis 0 Nov  5 10:20 file2
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:20 file3
$ chmod 700 permissions/file2
$ ls -l permissions/
total 0
-rwxrwxrwx 1 jarvis jarvis 0 Nov  5 10:20 file1
-rwx------ 1 jarvis jarvis 0 Nov  5 10:20 file2
-rw-r--r-- 1 jarvis jarvis 0 Nov  5 10:20 file3
$ chmod 755 permissions/file3
$ ls -l permissions/
total 0
-rwxrwxrwx 1 jarvis jarvis 0 Nov  5 10:20 file1
-rwx------ 1 jarvis jarvis 0 Nov  5 10:20 file2
-rwxr-xr-x 1 jarvis jarvis 0 Nov  5 10:20 file3
$ chmod a-r permissions/file1
$ ls -l permissions/
total 0
--wx-wx-wx 1 jarvis jarvis 0 Nov  5 10:20 file1
-rwx------ 1 jarvis jarvis 0 Nov  5 10:20 file2
-rwxr-xr-x 1 jarvis jarvis 0 Nov  5 10:20 file3
$ chmod a+x permissions/file2
$ ls -l permissions/
total 0
--wx-wx-wx 1 jarvis jarvis 0 Nov  5 10:20 file1
-rwx--x--x 1 jarvis jarvis 0 Nov  5 10:20 file2
-rwxr-xr-x 1 jarvis jarvis 0 Nov  5 10:20 file3
```

