## Router 객체로 라우팅 분리하기

### `next()`

- 다음 예의 경우 첫 번째 라우터의 첫 번째 미들웨어에서 `next('route')`를 호출해 두 번째, 세 번째 미들웨어는 실행이 되지 않고 두 번째 라우터로 넘어감

```js
router.get('/', function(req, res, next){
    next('route');
}, function(req, res, next) {
    console.log('실행되지 않습니다.');
    next();
}, function(req, res, next) {
    console.log('실행되지 않습니다.');
    next();
});
router.get('/', function(req, res) {
    console.log('실행됩니다');
    res.send('Hello, Express');
});
```

### 라우트 매개변수 패턴

```js
router.get('/user/:id', function(req, res) {
    console.log(req.params, req.query);
});
```

- `:id` 부분으로 `/users/1`이나 `/users/123` 등의 요청을 처리할 수 있음
- **일반 라우터보다 뒤에 위치해야 함**
    ```js
    router.get('/user/:id', function(req, res) {
        console.log('나만 실행');
    });
    router.get('/user/like', function(req, res) {
        console.log('실행 불가');
    });
    ```

- `/users/123?limit=5&skip=10`처럼 주소에 쿼리스트링은 `req.query` 객체 안에 다음과 같이 저장됨
    ```shell
    { id: '123' } { limit: '5', skip: '10' }
    ```

### 404 상태 코드 응답

```js
...

app.use((req, res, next) => {
  res.status(404).send('Not Found');
});

// 에러 처리 미들웨어 위에 넣는다
app.use((err, req, res, next) => {
  console.error(err);
  res.status(500).send(err.message);
});
```

### `router.route`

- 관련 있는 코드끼리 묶기

```js
router.route('/abc')
    .get((req, res) => {
        res.send('GET /abc');
    })
    .post((req, res) => {
        res.send('POST /abc');
    });
```
