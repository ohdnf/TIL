# git 명령어

bash 창에서 `git [명령어] -h`로 해당 명령의 도움말 확인 가능

| 명령 | 내용 | 비고 |
| ---- | ---- | ---- |
| `git init` | 현재 폴더에 Git Repository 생성 |      |
| `git status` | git 현재 상태 확인 |      |
| `git config -l` | config 설정 확인하기 |      |
| `git add` | 파일 또는 폴더를 staging |      |
| `git add -u` | 수정사항을 모두 add |      |
| `git commit` | staging area에 있는 파일들을 local repository에 저장(commit message를 입력하기 위한 창이 뜬다) | |
| `git commit -m 'Commit message'` | 커밋 메시지와 함께 저장 |      |
| `git log --oneline` | git log 한줄만 확인 |      |
| `git log --oneline --graph` | git log graph로 표시 |      |
| `git checkout <브랜치명>` | 해당 커밋(+브랜치)으로 버전 변경 |      |
| `git checkout -b <브랜치명>` | 새로운 커밋(+브랜치)으로 버전 변경 |      |
| `git remote` | 원격저장소와 통신(목록 표시) |      |
| `git remote add <원격저장소_별명> <원격저장소_URL>` | 원격저장소 추가(기본 별명: `origin`)  |      |
| `git push <원격저장소_별명> <브랜치명>` | 브랜치(`master`)를 원격저장소에 저장 |      |
| `git clone <원격저장소_URL>` | 원격저장소를 복제(`git init` 필요 X)   |      |
| `git clone <원격저장소_URL> <원격저장소_별명>` | 내가 정한 별명으로 clone |      |
| `git pull <원격저장소_별명> <브랜치명>` | 원격저장소의 변경 내용을 끌어오기 |      |
| `git branch <원격저장소_별명>` | 브랜치 생성 |      |
| `git branch -d <원격저장소_별명>` | 브랜치 삭제 |      |
| `git switch <브랜치명>` | 브랜치 이동 |      |
| `git switch -c <브랜치명>` | 새로운 브랜치를 만들고 해당 브랜치로 이동 |      |
| `git merge <브랜치명>` | `master` 브랜치에서 merge하려면 `master`로 이동한 후 merge할 브랜치를 명령 뒤 매개변수로 입력 |      |
| `git rm --cached <file>` | git 인덱스에서 track하던 file을 삭제 | [Stack Overflow](https://stackoverflow.com/questions/1274057/how-to-make-git-forget-about-a-file-that-was-tracked-but-is-now-in-gitignore) |
| `git rm -r --cached <folder>` | git 인덱스에서 track하던 folder를 삭제 |      |
| `git stash` | Working directory에서 수정한 파일들(Modified이면서 Tracked 상태인 파일)을 **commit하지 않고** 보관(stack 구조로 저장) | [Stashing과 Cleaning](https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-Stashing%EA%B3%BC-Cleaning) |
| `git stash list` | stash를 한 상태 목록을 보여준다 |  |
| `git stash apply [stash이름]` | 리스트 가장 위에 있는 stash 적용 |  |
| `git stash apply --index` | stash할 때 staged 상태였던 파일들을 staged 상태로 적용해주는 옵션 |  |
| `git stash drop` | apply는 현재 stash 스택의 top만 **단순히 적용만** 한다. drop 옵션을 사용해 top stash를 제거한다. |  |
| `git stash pop` | `git stash apply & git stash drop`과 동일하다. |  |



## git을 활용한 협업 기초

1. `master`에 **commit X**
2. 항상 작업 시작 전에 본인의 `branch` 확인
3. ~~`merge`된 `master`를 `pull` 이후에는 본인 `branch` 삭제 후 재생성~~
4. 본인의 `branch`에서 `commit`하고 팀원들과 공유하는 원격 저장소(예. GitHub)에 `pull request`로 코드 리뷰
5. 1-4 반복



## branch

branch는 **일회용**이다.

branch는 feature를 develop하기 위해 만드는 경우가 대다수

`master`에 `merge` 한 후 삭제해주자



## merge

1. FF(Fast-Forward) Merge

    - commit msg 추가 가능

2. Auto Merge

    개별 branch 작업들이 충돌하지 않을 때 git이 자동으로 수행

3. Conflict



## gitlab 인증 초기화?

깃랩 원격 저장소의 인증을 초기화하는 방법

[더 알아보기](https://git-scm.com/book/ko/v2/Git-%EB%8F%84%EA%B5%AC-Credential-%EC%A0%80%EC%9E%A5%EC%86%8C)

```shell
$ git credential reject
protocol=https
host=lab.ssafy.com
```

