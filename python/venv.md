# 가상 환경

Python 표준 라이브러리에 제공되지 않는 패키지와 모듈을 사용할 때, 또는 각 프로젝트에서 서로 다른 버전을 사용하기 위해 쓰는 설정입니다.



## 가상 환경 만들기

원하는 디렉터리에서 `venv` 모듈을 스크립트로 실행합니다.

```bash
# 현재 디렉터리에 my-venv라는 가상 환경 생성
# 일반적으로 my-venv보다는 venv, env로 설정합니다.
# 설정의 이해를 돕기 위해 my-venv로 이름을 지었습니다.
python -m venv my-venv
```



> git을 사용하여 버전 관리를 하는 경우, `.gitignore` 파일에 `venv`를 추가해 추적하지 않도록 설정해야 합니다. git 원격저장소에는 다같이 공유하는 만큼 최소한으로 필요한 파일만 저장되어야 합니다. 프로젝트 실행을 위해 필요한 패키지나 모듈이 있다면 `requirements.txt` 파일에 명시하도록 합니다.
>
> ```bash
> # gitignore 설정
> $ echo "my-venv" >> .gitignore
> 
> # 가상 환경 실행 상태에서 설치된 패키지와 모듈 목록 저장
> $ pip freeze > requirements.txt
> 
> # 새로운 가상 환경에서 패키지 및 모듈 의존성 설치
> $ pip install -r requirements.txt
> ```
>



## 가상 환경 실행하기

### 윈도우에서 실행

```shell
> my-venv\Scripts\activate.bat
```

### Linux 또는 Mac에서 실행

```shell
$ source my-venv/bin/activate
```

또는 `.bashrc`에 `alias` 설정을 할 수 있습니다.

```bash
# .bashrc
alias activate="source my-venv/bin/activate"
```



## 가상 환경 종료하기

```bash
# Windows
> deactivate
# Linux
$ deactivate
```

