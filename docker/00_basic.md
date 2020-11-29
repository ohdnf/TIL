# Docker 입문



## 설치

> Linux Ubuntu



## 초기 설정

### `sudo` 권한

> `docker` 명령어는 `root` 계정 권한으로 실행해야 하기 때문에, 일반 계정으로 명령어를 칠 경우 `sudo`를 생략하고 싶을 때 해당 계정을 `docker` 그룹에 추가한다. 계정 로그아웃 후 다시 로그인해야 반영된다!

```shell
$ sudo usermod -aG docker ${USER}
$ sudo service docker restart
$ logout
```



