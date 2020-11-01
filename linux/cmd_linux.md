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
| `apt-get install <패키지 이름>` | 패키지 설치 |
| `apt-get update` | 패키지 업그레이드 가능 여부 체크 |
| `apt-get upgragde` | 업그레이드 가능한 패키지 업그레이드 |
|  |  |



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