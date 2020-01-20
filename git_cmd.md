# git 명령어



| 명령                                      | 내용                                 | 비고 |
| ----------------------------------------- | ------------------------------------ | ---- |
| `git init`                                | 현재 폴더에 Git Repository 생성      |      |
| `git status`                              | git 현재 상태 확인                   |      |
| `git config -l`                           | config 설정 확인하기                 |      |
| `git add`                                 | 파일 또는 폴더를 staging             |      |
| `git add -u`                              | 수정사항을 모두 add                  |      |
| `git commit -m '커밋 메시지'`             | 커밋 메시지와 함께 저장              |      |
| `git log --oneline`                       | git log 한줄만 확인                  |      |
| `git checkout [브랜치_주소]`              | 해당 커밋으로 버전 변경              |      |
| `git remote`                              | 원격저장소와 통신(목록 표시)         |      |
| `git remote add [원격_별명] [원격_URL]`   | 원격저장소 추가(기본: `origin`)      |      |
| `git push [원격_별명] [브랜치]`           | 브랜치(`master`)를 원격저장소에 저장 |      |
| `git clone [원격_URL]`                    | 원격저장소를 복제(`git init` 필요 X) |      |
| `git clone [원격_URL] [사용자_지정_별명]` | 내가 정한 별명으로 clone             |      |
| `git pull [원격] [브랜치]`                | 원격저장소의 변경 내용을 끌어오기    |      |
|                                           |                                      |      |



git 초기화?

```shell
$ git credential reject
protocol=https
host=lab.ssafy.com


```

