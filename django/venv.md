## How

1. Create pjt dir
2. Add `venv/` to `.gitignore`
3. `$ python -m venv {virtual_env_name}`
    > 가상환경 이름은 보통 `venv`로 한다..
4. `$ source {virtual_env_name}/bin/activate` or `$ source {virtual_env_name}/Scripts/activate`
    > 전자는 linux, 후자는 windows OS 환경
5. `$ pip install django`
    > 필요한 pip 추가
6. `$ pip freeze > requirements.txt`
    > 설치한 pip를 필수 설치 요소로 문서화

## 협업 시 프로젝트 설정 Steps
1. `git clone github.com/~`
2. `.gitignore`에 `venv/` 추가
3. `$ python -m venv venv`
4. `$ source venv/bin/activate`
5. `$ pip install -r requirements.txt`
6. 개발 시작

## alias

```bash
# .bashrc
alias va="source venv/bin/activate"
```

```shell
$ source ./bashrc
```