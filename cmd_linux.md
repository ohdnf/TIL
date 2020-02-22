# linux 명령어



| 명령어 | 내용 |
| ----------------------------------- | -------------------------------------------------- |
| `pwd`                               | 현재 작업 디렉토리 표시 |
| `ls -al`                            | 현재 작업 디렉토리에 있는 디렉토리 및 파일 표시 |
| `cd [dir_path]`                     | 디렉토리 이동(`.` : 현재 디렉토리 / `..` 상위 디렉토리) |
| `mkdir [dir_name]`                  | 디렉토리 생성 |
| `rm -r`                             | 디렉토리 삭제 (폴더 내 파일 Recursive하게 삭제) |
| `touch a.txt`                       | 파일 `a.txt` 생성 |
| `mv [file_path] [destination]`      | 파일/폴더를 해당 위치로 이동(이름 변경 가능) |
| `cp [file_path] [destination]`      | 복사 대상 경로 파일을 복사 위치로 복사(이름 변경 가능) |
| `cat (-n) [file_path]`              | (옵션: `n`행만큼) 파일 내용 출력 |
| `cat [file_path] > [destination]`   | 기존 파일 내용을 다른 파일로 복사 |
| `echo '[contents]' > [file_path]`   | 해당 파일 내용을 `[contents]`로 덮어쓰기 |
| `echo '[contents]' >> [file_path]`  | 해당 파일 맨 뒤에 `[contents]` 이어쓰기 |
| `code`                              | `code .` : 현재 디렉토리에서 vscode 실행 |
| `python`                            | Python으로 파일 실행(ex. `python hello.py`) |
| `pip`                               | Python Package Installer |



## linux cmd 단축키 만들기

root 디렉토리에 `.bashrc` 파일을 만들어준다.

```shell
$ vi ~/.bashrc
```

```vi
alias jn="jupyter notebook"
alias inout="mv ~/Downloads/input.txt ~/Downloads/output.txt ."
```

> 띄어쓰기 조심!

`Esc` 눌러서 일반 모드로 나오고 `:wq` 누르고 `Enter` 눌러서 저장 후 종료

이 상태에선 설정이 저장되지 않으므로 **터미널 새로고침**이 필요

```shell
$ source ~/.bashrc
```
