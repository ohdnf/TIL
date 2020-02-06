# git 명령어

bash 창에서 `git [명령어] -h`로 해당 명령의 도움말 확인 가능



| 명령                                      | 내용                                 | 비고 |
| ----------------------------------------- | ------------------------------------ | ---- |
| `git init`                                | 현재 폴더에 Git Repository 생성       |      |
| `git status`                              | git 현재 상태 확인                    |      |
| `git config -l`                           | config 설정 확인하기                  |      |
| `git add`                                 | 파일 또는 폴더를 staging              |      |
| `git add -u`                              | 수정사항을 모두 add                   |      |
| `git commit -m 'Commit message'`          | 커밋 메시지와 함께 저장                |      |
| `git log --oneline`                       | git log 한줄만 확인                   |      |
| `git log --oneline --graph`               | git log graph로 표시                 |      |
| `git checkout [branch]`                   | 해당 커밋(+브랜치)으로 버전 변경       |      |
| `git checkout -b [branch]`                | 새로운 커밋(+브랜치)으로 버전 변경      |      |
| `git remote`                              | 원격저장소와 통신(목록 표시)           |      |
| `git remote add [alias] [remote_URL]`     | 원격저장소 추가(기본 alias: `origin`)  |      |
| `git push [remote] [branch]`              | 브랜치(`master`)를 원격저장소에 저장   |      |
| `git clone [remote_URL]`                  | 원격저장소를 복제(`git init` 필요 X)   |      |
| `git clone [remote_URL] [alias]`          | 내가 정한 별명으로 clone               |      |
| `git pull [remote] [branch]`              | 원격저장소의 변경 내용을 끌어오기       |      |
| `git branch [alias]`                      | 브랜치 생성                           |      |
| `git branch -d [alias]`                   | 브랜치 삭제                           |      |
| `git switch [branch]`                     | 브랜치 이동                           |      |
| `git switch -c [branch]`                  | 새로운 브랜치를 만들고 해당 브랜치로 이동 |      |
| `git merge [branch]`                      | `master` 브랜치에서 merge하려면 `master`로 이동한 후 merge할 브랜치를 명령 뒤 매개변수로 입력                                              |      |
|           |           |       |



## golden_bell(공동 작업)

1. `master`에 **commit X**

2. 항상 작업 시작 전에 본인의 `branch`(내 경우 `ohdnf`) 확인

3. `merge`된 `master`를 `pull` 이후에는 본인 `branch` 삭제 후 재생성

4. 1-3 반복



## branch

branch는 **일회용**이다.

branch는 feature를 develop하기 위해 만드는 경우가 대다수



## merge

1. FF(Fast-Forward) Merge

    - commit msg 추가 가능

2. Auto Merge

    개별 branch 작업들이 충돌하지 않을 때 git이 자동으로 수행

3. Conflict


### git 초기화?

검색해볼 것...

```shell
$ git credential reject
protocol=https
host=lab.ssafy.com


```

