#git

`git init`
현재 디렉토리에서 git 버전 관리를 시작



`git status`
현재 디렉토리의 버전 현황



`git add`
버전 관리할 파일을 `staging area`로 이동. `staging area`에서 tracking되어야만 commit할 수 있다.
```
> git add --help
> git add .
> git add -A
```



`git commit`
버전을 만든다.
```
> git commit --help
> git commit -m 'Commit message'
> git commit -am 'add + commit'
> git commit
```
* With `-am` option, a version to be commited is needed to be tracked. It means it has to be 'added' at the very first time(`-am` 옵션은 `git add`와 `git commit`을 한꺼번에 해준다. 하지만 해당 파일이나 디렉토리가 tracked, 즉 최초 한 번은 add되어 git이 추적가능한 상태여야 한다.)
* Just type only `git commit`, you can type multi line for commit message. In this case you need set text editor with `git config --global core.editor "TEXT_EDITOR"` TEXT_EDITOR can be vim, nano, etc. (`git commit`만 칠 경우 여러 줄로 commit message를 작성할 수 있다. `git config`에서 내가 익숙한 텍스트 에디터를 기본 설정으로 변경할 수 있다.)



`git log`
버전 기록을 보여준다.
```
> git log --help
> git log
기본 commit log 표시
> git log --stat
insertions, deletions 표시
> git log -p
상세 수정 사항 표시
```



`git diff`
Show changes between commits, commit and working tree, etc


`git checkout`
`git log`를 통해 branch ID를 확인할 수 있다. `git checkout branch_ID`를 하면 해당 버전으로 변경할 수 있다.


`git reset`
버전을 삭제하는 명령어
```
> git reset --help
> git reset --hard
```



`git revert`
버전을 되돌리는 명령어. 스택 구조(FIFO) 방식으로 버전 역행이 가능하다. 현재 버전 4까지 있고 버전 2로 가고 싶은 경우: `git revert 4` -> `git revert 3`