# git

## git이란?

`SCM(Source Code Management)`, `VCS(Version Control System)`

* Versioning을 통해 Source Code를 Management하는 Tool
* "Folder-based" management
* `Version` == `Snapshot`



## git을 쓰는 이유

* 버전 관리
* 원격 저장
* 협업



## git area

    | Working tree      | Where `.git` directory exist. **Untracked** area of git. |
    | Staging area      | Changes moved from working tree by `git add` cmd. |
    | Local repository  | Everything in `.git` directory. Changes at staging area can be added to `.git` directory by running cmd `git commit`. |
    | Remote repository | Local repository can be saved and shared by SCM platforms such as github, gitlab, bitbucket, etc. |



## init version control

- `git init`: 현재 디렉토리에서 git 버전 관리를 시작



- `git status`: 현재 디렉토리의 버전 현황



- `git add`: 버전 관리할 파일을 `staging area`로 이동. `staging area`에서 `tracking`되어야만 `commit`할 수 있다.

    ```shell
    $ git add --help
    $ git add .
    $ git add -A
    ```



- `git commit`: 버전을 만든다.

    ```shell
    $ git commit --help
    $ git commit -m 'Commit message'
    $ git commit -am 'add + commit'
    ```

    - `-am` 옵션은 `git add`와 `git commit`을 한꺼번에 해준다. 하지만 해당 파일이나 디렉토리가 tracked, 즉 최초 한 번은 add되어 git이 추적가능한 상태여야 한다.(With `-am` option, a version to be commited is needed to be tracked. It means it has to be 'added' at the very first time)

    - `git commit`만 칠 경우 여러 줄로 commit message를 작성할 수 있다. `git config`에서 내가 익숙한 텍스트 에디터를 기본 설정으로 변경할 수 있다.(Just type `git commit`, you can type multi line for commit message. In this case you need set text editor with `git config --global core.editor "TEXT_EDITOR"` TEXT_EDITOR can be vim, nano, etc.)

    - 최근 commit message를 수정
    ```shell
    $ git commit --amend
    ```



- `git log`: 버전 기록을 보여준다.

    ```shell
    $ git log --help
    $ git log              # 기본 commit log 표시
    $ git log --stat       # insertions, deletions 표시
    $ git log -p           # 상세 수정 사항 표시
    $ git log --oneline    # log 메시지를 한 줄씩 표시
    $ git log --graph      # branch를 그래프로 표시
    ```



- `git diff`: Show changes between commits, commit and working tree, etc



- `git reset`: 버전을 삭제하는 명령어

    ```shell
    $ git reset --help
    $ git reset --hard
    ```



- `git revert`: 버전을 되돌리는 명령어. 스택 구조(FIFO) 방식으로 버전 역행이 가능하다. 
    > 현재 버전 4까지 있고 버전 2로 가고 싶은 경우: `git revert 4` -$ `git revert 3`



## Branch & Conflict

- `git branch`

    ```shell
    $ git branch
    * master
    브랜치 목록 보기. 기본 브랜치는 master
    $ git branch apple
    기존에 apple이라는 브랜치가 없었다면, apple 이라는 새로운 브랜치 생성
    $ git branch
    apple
    * master
    ```



- `git checkout`: HEAD를 변경해 현재 저장소 위치를 바꾸는 명령어



- `git log`를 통해 branch ID를 확인할 수 있다. `git checkout branch_ID`를 하면 해당 버전으로 변경할 수 있다.

    ```shell
    $ git branch
    apple
    * master
    $ git checkout apple
    apple 브랜치로 이동
    ```

- HEAD는 브랜치가 아닌 버전을 가리킬 수도 있다. 이런 상태를 `detached`라고 한다.



- `git merge`: 서로 다른 브랜치를 병합

    * master 브랜치와 custom 브랜치(예시: apple)를 병합
    * master 브랜치에서 병합할 브랜치 이름을 `git merge` 명령어 뒤에 붙인다.(merge apple into master)

        ```shell
        $ git branch
        apple
        * master
        $ git merge apple
        ```

- 3-way merge

    ```
    base    here	there	2way_m  3way_m
    ----------------------------------------------------
    A	    A	    A	    A	    A
    H	    B   	B   	?   	H
    C	    C	    T	    ?	    T
    H       D   	T   	?   	?
    ```

    * `git mergetool` 을 사용해서 merge 관리(예: p4merge)


## Remote repository

### 혼자 작업하기

    ```shell
    $ git remote add origin git@github.com:~
    $ git push -u origin master
    ```

* 새로운 원격저장소를 만들고, 지역저장소와 연결하려면 git cli에서 `git remote add` 명령어를 쓴다. 뒤에 원격저장소 HTTP 주소가 필요하다.

* 원격저장소를 추가한 뒤 `git push` 명령어를 통해 지역저장소 버전을 원격저장소에 업로드할 수 있다. 이 때, 맨처음 push할 때 `-u` 옵션을 줘야 원격저장소의 master 브랜치와 지역저장소의 master 브랜치를 연결할 수 있다.



### 같이 작업하기(너 내 동료가 돼라!)

* github의 원격저장소 > `Setting` > `Collaborator`에 함께 작업할 동료 아이디 또는 이메일 추가
* 원격저장소의 내용을 `git pull`을 통해서 다른 사람이 작업한 내용을 가져오고 정리해 타임라인을 만든 다음 작업을 시작
* `CONFLICT`가 발생하면 `git merge` 등을 통해 해결해야한다.