# Python 정규식 HOWTO

> Python `re` 모듈을 사용해 정규식을 사용하는 방법
>
> https://docs.python.org/ko/3/howto/regex.html
> https://wikidocs.net/4308

## 메타 문자

다음 메타 문자들은 특별한 의미를 갖습니다.

```
. ^ $ * + ? { } [ ] \ | ( )
```

### 문자 클래스 `[ ]`

`[ ]` 사이 문자들 중 하나와 매치
