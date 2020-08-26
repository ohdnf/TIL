# 가상 환경

표준 라이브러리에 제공되지 않는 패키지와 모듈을 사용할 때, 서로 다른 버전을 사용하기 위해 쓰는 설정



## 가상 환경 만들기

원하는 디렉터리에서 `venv` 모듈을 스크립트로 실행

```sh
# 현재 디렉터리에 my-venv라는 가상 환경 생성
python3 -m venv my-venv
```



## 가상 환경 실행하기

윈도우에서 실행

```shell
my-venv\Scripts\activate.bat
```

Unix 또는 MacOS에서 실행

```shell
source my-venv/bin/activate
```