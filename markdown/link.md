# 목차 만들기

아래와 같이 목차를 만들었을 때 목차 목록에 있는 헤더를 누르면 페이지 아래의 해당 헤더로 바로 가게 하고 싶다.

```markdown
# 목차입니다.
## 첫 번째 주제
### 첫 번째 내용
### 두 번째 내용
## 두 번째 주제
### 세 번째 내용
### 네 번째 내용
...

# 첫 번째 주제
## 첫 번째 내용
...
```



이를 위해서 목차의 각 헤더에 다음과 같이 링크를 지정해주면 된다.

```markdown
# 목차입니다.
## [첫 번째 주제](#첫-번째-주제)		--> 클릭하면
### [첫 번째 내용](#첫-번째-내용)
### 두 번째 내용
## 두 번째 주제
### 세 번째 내용
### 네 번째 내용
...

# 첫 번째 주제						<-- 여기로 스크롤 이동
## 첫 번째 내용
...
```



헤더 링크 ID(`()`안에 들어갈 `#`로 시작하는 텍스트)엔 규칙이 존재하는데, 다음과 같다.

- 모든 텍스트는 소문자로 전환한다.
- 문장부호(`,`, `.`, `_`, `:` 등)나 이모티콘 등 비언어적 텍스트는 제거한다.
- 모든 띄어쓰기는 `-`(하이픈)으로 대체한다.
- 두 개 이상 연속되는 하이픈은 하나로 대체한다.
- 헤더가 같다면 뒤에 숫자로 구분한다(자동으로 생성, 1부터 시작)



예시를 보자([참고](https://docs.gitlab.com/ee/user/markdown.html#header-ids-and-links))

```markdown
# This header has spaces in it
## This header has a :thumbsup: in it
# This header has Unicode in it: 한글
## This header has spaces in it
### This header has spaces in it
## This header has 3.5 in it (and parentheses)
```

위 헤더의 링크 ID는 다음과 같이 생성된다.

```markdown
this-header-has-spaces-in-it
this-header-has-a-in-it
this-header-has-unicode-in-it-한글
this-header-has-spaces-in-it-1
this-header-has-spaces-in-it-2
this-header-has-3-5-in-it-and-parentheses
```

