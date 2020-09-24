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
        - `next()`는 다음 미들웨어로
        - `next('route')`는 바로 다음 라우터로
        - `next(error)`는 바로 에러 처리 미들웨어로
- 미들웨어 간 데이터 넘기기
    - `req`에 객체를 넣어 사용
        ```js
        app.use((req, res, next) => {
            req.data = '데이터 넣기';   // 속성명이 꼭 data일 필요는 없다
            next()
        }, (req, res, next) => {
            console.log(req.data); // 데이터 받기
            next();
        });


### 에러 처리 미들웨어

```js
app.use((err, req, res, next) => {
  console.error(err);
  res.status(500).send(err.message);
});
```

- 매개변수가 `err, req, res, next`로 모든 매개변수를 사용하지 않더라도 반드시 네 개 작성

### 미들웨어 안에 미들웨어 넣기

```js
app.use(morgan('dev'));
// 동일
app.use((req, res, next) => {
    morgan('dev')(req, res, next);
});
```

- 기존 미들웨어의 기능을 확장할 때 사용
- 아래와 같이 분기 처리할 때 사용

```js
app.use((req, res, next) => {
    if (process.env.NODE_ENV === 'production') {
        morgan('combined')(req, res, next);
    } else {
        morgan('dev')(req, res, next);
    }
});
```

### 자주 쓰이는 미들웨어

#### morgan

요청과 응답에 대한 정보를 콘솔에 기록

```js
const morgan = require('morgan');

app.use(morgan('dev')); // combined, common, short, tiny
// 출력: [HTTP 메서드] [주소] [HTTP 상태 코드] [응답 속도] - [응답 바이트]
// GET / 500 19.024 ms - 50
```

#### static

- 정적인 파일들을 제공하는 라우터 역할
- express 객체에서 기본적으로 제공

```js
// app.use('요청 경로', express.static('실제 경로'));
app.use('/', express.static(path.join(__dirname, 'public')));
```

`public/stylesheets/style.css`는 `http://localhost:3000/stylesheets/style.css`로 접근 가능

#### body-parser

- 요청의 본문에 있는 데이터를 해석해서 `req.body` 객체로 만들어주는 미들웨어
- 보통 폼 데이터나 AJAX 요청의 데이터를 처리

```js
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
```

- 익스프레스 4.16.0 버전부터 body-parser 미들웨어의 일부 기능이 익스프레스에 내장
- JSON과 URL-encoded 형식의 데이터 외 **버퍼나 텍스트 요청을 처리**할 필요가 있다면 body-parser를 설치한 후 다음과 같이 추가 필요
    ```js
    const bodyParser = require('body-parser');
    app.use(bodyParser.raw());
    app.use(bodyParser.text());
    ```
- URL-encoded?
    - 주소 형식으로 데이터를 보내는 방식
    - 폼 전송에서 주로 사용
    - `extended?`
        - `false`: `querystring` 모듈을 사용해 쿼리스트링 해석(내장 모듈)
        - `true`: `qs` 모듈이라는 npm 패키지를 사용해 쿼리스트링 해석(추가 설치 필요). 확장 기능

- `req.on('data')`, `req.on('end')`로 스트림 처리하지 않아도 내부적으로 `req.body`에 추가
    - JSON 형식으로 `{ name: 'ohdnf', language: 'nodejs' }`를 본문으로 보내면 `req.body`에 그대로 반영
    - URL-encoded 형식으로 `name=ohdnf&language=nodejs`를 본문으로 보내면 `req.body`에 `{ name: 'ohdnf', language: 'nodejs' }`가 반영

#### cookie-parser

- 요청에 동봉된 쿠키를 해석해 `req.cookies` 객체로 만듦

```js
const cookieParser = require('cookie-parser');

app.use(cookieParser(비밀키));
```

- 비밀 키를 통해 해당 쿠키가 내 서버가 만든 쿠키임을 검증
- 서명된 쿠키는 `req.cookies` 대신 `req.signedCookies` 객체에 들어감
- 쿠키 생성/제거는 다른 메서드 사용
    - 쿠키 생성
        ```js
        res.cookie(키, 값, 옵션)
        ```
    - 쿠키 삭제
        ```js
        res.clearCookie(키, 값, 옵션)
        ```

#### express-session

- 로그인과 같은 세션 구현, 관리 등에 사용
- 세션은 `req.session` 객체 안에 유지

```js
app.use(session({
  resave: false,    // 수정사항 없어도 다시 세션 저장
  saveUninitialized: false, // 세션에 저장할 내역이 없어도 다시 세션 생성
  secret: process.env.COOKIE_SECRET,
  cookie: {
    httpOnly: true, // 클라이언트에서 쿠키 확인 불가
    secure: false,  // https가 아닌 환경에서 사용 가능
  },
  name: 'session-cookie',
}));
```

- `1.5` 버전 이전에는 내부적으로 `cookie-parser`를 사용해 `cookie-parser` 미들웨어 뒤에 위치시켜야 함
- 세션 관리 시 클라이언트에 **세션 쿠키**를 보낸다. 서명을 위해 비밀 키를 설정(`cookie-parser`와 같은 값을 사용 권장)
- 배포 시에는 `store`라는 옵션에 DB(보통 Redis 사용)를 연결하여 서버가 재시작되어도 세션을 유지할 수 있도록 설정하는 것이 좋다.

```js
req.session.name = 'ohdnf'; // 세션 등록
req.sessionID;              // 세션 아이디 확인
req.session.destroy();      // 세션 모두 제거
```

#### multer

- 이미지, 동영상 등을 비롯한 여러 가지 파일들을 멀티파트 형식으로 업로드할 때 사용하는 미들웨어
- 멀티파트 형식
    - `enctype="multipart/form-data"`인 폼을 통해 업로드하는 데이터 형식

```js
// app.js

const multer = require('multer');
const fs = require('fs');
try {
  fs.readdirSync('uploads');
} catch (error) {
  console.error('uploads 폴더가 없어 uploads 폴더를 생성합니다.');
  fs.mkdirSync('uploads');
}
const upload = multer({
  storage: multer.diskStorage({
    destination(req, file, done) {
      done(null, 'uploads/');
    },
    filename(req, file, done) {
      const ext = path.extname(file.originalname);
      done(null, path.basename(file.originalname, ext) + Date.now() + ext);
    },
  }),
  limits: { fileSize: 5 * 1024 * 1024 },
});
app.get('/upload', (req, res) => {
  res.sendFile(path.join(__dirname, 'multipart.html'));
});
app.post('/upload',
  upload.fields([{ name: 'image1' }, { name: 'image2' }]),
  (req, res) => {
    console.log(req.files, req.body);
    res.send('ok');
  },
);
```

```html
<!-- multipart.html -->

<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>멀티파트 형식 업로드</title>
</head>
<body>
  <form action="/upload" method="post" enctype="multipart/form-data">
    <input type="file" name="image1" id="img1" />
    <input type="file" name="image2" id="img2" />
    <input type="text" name="title" id="title" />
    <button type="submit">업로드</button>
  </form>
</body>
</html>
```