# Learning Express.js

## Middleware

- 요청과 응답의 중간에 위치
- 라우터, 에러 핸들러 등

### 사용 형태

- `app.use(미들웨어)`의 형태로 사용
    - `app.use(미들웨어)`
        모든 요청에서 미들웨어 실행
    - `app.use('/abc', 미들웨어)`
        abc로 시작하는 요청에서 미들웨어 실행
    - `app.post('/abc', 미들웨어)`
        abc로 시작하는 POST 요청에서 미들웨어 실행
- `app.use((req, res, next) => {})`
    - `next`는 다음 미들웨어로 넘어가는 함수

### 에러 처리 미들웨어

```js
app.use((err, req, res, next) => {
  console.error(err);
  res.status(500).send(err.message);
});
```

- 매개변수가 `err, req, res, next`로 모든 매개변수를 사용하지 않더라도 반드시 네 개 작성