
## RESTful API

> URL을 깔끔하게 정리하는 방식(Convention)
> [참고 링크](https://meetup.toast.com/posts/92)

- C: new, create, write, make, render...
- R: index(전체), detail(상세), '', read, show...
- U: ... 
- D: ...

### 1. 동사를 URL에 넣지말자!

- C: POST
- R: GET
- U: PUT
- D: DELETE

> HTTP Method(verb) 활용!

### 2. 목적어(Resource)만 URL에 넣자!

Resource == Data

- C: (POST)     /articles
- R: (GET)
    - index =>  /articles
    - detail => /articles/<id>
- U: (PUT)      /articles/<id>
- D: (DELETE)   /articles/<id>

### API 관련 URL
1. subdomain
```
lab.ssafy.com
api.github.com
```

2. 분리 URL /api/
```
ssafy.com/api/lectures/
github.com/api/repos/
```

3. Versioning
```
ssafy.com/api/v1/lectures/
```

POST /api/v1/articles/1/like/
POST /api/v2/aritcles/1/comments/like/


/api/v1/articles/
/api/v1/articles/<id>/

