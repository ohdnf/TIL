
# RESTful API

## REST?

**RE**presentational **S**tate **T**ransfer

> REST is an architecture style for designing networked application.

### REST 구성

- URI: 정보의 자원(Resource)을 표현(동사 사용 지양 / 복수 명사 사용)

  ```
  GET /members/delete/1	(X)
  ```

- HTTP Methods: 자원에 대한 행위(Verb)를 표현

  ```
  DELETE /members/1		(O)
  ```

- 표현(Representations)

## RESTful API

- Web HTTP Protocol
- URI(Uniform REsource Identifiers)로 리소스(자원)를 표현
- URI는 단순하고 직관적인 구조
- 리소스의 상태는 HTTP Methods를 활용해 구분
- xml/json을 활용해 데이터 전송



## REST API 디자인 가이드

### CRUD

리소스를 다루기 위한 행위들. 각 행위에 대응되는 HTTP Methods가 존재

- Create: POST
- Retrieve: GET
- Update: PUT
- Delete: DELETE

> URL을 깔끔하게 정리하는 방식(Convention)
> [참고 링크](https://meetup.toast.com/posts/92)
>
> | Method | 역할                              |
> | ------ | --------------------------------- |
> | POST   | URI를 요청하면 해당 리소스를 생성 |
> | GET    | 해당 리소스를 조회                |
> | PUT    | 해당 리소스를 수정                |
> | DELETE | 해당 리소스를 삭제                |
>
> 예시)
>
> | URL              | HTTP Method | 설명                      |
> | ---------------- | ----------- | ------------------------- |
> | `/movies`        | GET         | 모든 영화 리스트 가져오기 |
> | `/movies`        | POST        | 영화 추가                 |
> | `/movies/:title` | GET         | title 해당 영화 가져오기  |
> | `/movies/:title` | DELETE      | title 해당 영화 삭제      |
> | `/movies/:title` | PUT         | title 해당 영화 수정      |
> | `/movies?min=9`  | GET         | 상영 중인 영화 리스트     |





### 목적어(Resource)만 URL에 넣자!

> Resource == Data

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

