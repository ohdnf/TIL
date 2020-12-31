# Middlewares

> 애플리케이션의 request-response 사이클 안에서 request 객체, response 객체, 그리고 next 함수에 접근 가능한 함수이다.
>
> ~~처리가 끝날 때까지 연결되어 있는 것~~

## Basics

```javascript
const express = require('express')
const app = express()

const PORT = 4000;

function handleListening() {
  console.log(`Listening on: http://localhost:${PORT}`);
}

const handleHome = (req, res) => {
  res.send('hello from home');
}

const handleProfile = (req, res) => {
  res.send('You are on my profile')
}

const middleware = (req, res, next) => {
  console.log("I am between user request and app response");
  next();
}

app.get('/', middleware, handleHome);

app.get('/profile', middleware, handleProfile);

app.listen(PORT, handleListening);

```

middleware의 인자로 `req`, `res`, `next`가 있는데, `next`는 다음 콜백 함수로 넘어가게 해준다. `handleHome`, `handleProfile`과 같은 마지막 함수에는 `next`가 필요없다. 또한 미들웨어 안에서 `res.send()` 함수를 통해서 다음 함수로 넘어가지 않고 라우터의 실행을 종료시킬 수도 있다.

middleware는 위치가 상당히 중요하다.

```javascript
const express = require('express')
const app = express()

const PORT = 4000;

function handleListening() {
  console.log(`Listening on: http://localhost:${PORT}`);
}

const handleHome = (req, res) => {
  res.send('hello from home');
}

const handleProfile = (req, res) => {
  res.send('You are on my profile')
}

const middlewareGlobal = (req, res, next) => {
  console.log("I am between every user requests and app responses");
  next();
}

const middlewareForHome = (req, res, next) => {
  console.log("루트페이지에 접속했을 때만 실행되는 미들웨어");
  next();
}

const middlewareAfterHome = (req, res, next) => {
  console.log("루트페이지 이후 설정된 라우터에 대해서만 실행되는 미들웨어");
  next();
}

const middlewareForProfile = (req, res, next) => {
  console.log("프로필 페이지를 위한 미들웨어");
  next();
}

// 모든 사용자 요청과 그에 대한 응답 사이에서 실행되는 미들웨어
app.use(middlewareGlobal);

app.get('/', middlewareForHome, handleHome);

// 미들웨어는 위치가 굉장히 중요하다.
app.use(middlewareAfterHome);

app.get('/profile', middlewareForProfile, handleProfile);

app.listen(PORT, handleListening);

```



## 설치 및 실행

`npm install 미들웨어명` 명령어를 통해 설치하고 아래 문법을 사용해 실행한다.

### `app.use()`

| 문법                         | 내용                                       |
| ---------------------------- | ------------------------------------------ |
| `app.use(미들웨어)`          | 모든 요청에서 미들웨어 실행                |
| `app.use('/abc', 미들웨어)`  | abc로 시작하는 요청에서 미들웨어 실행      |
| `app.post('/abc', 미들웨어)` | abc로 시작하는 POST 요청에서 미들웨어 실행 |

### 자주 사용하는 미들웨어

- `morgan` logging을 위한 미들웨어
  - `combined`, `common`, `dev`, `short`, `tiny` 설정으로 log 출력을 변경할 수 있다.
- `helmet` Node.js 보안을 도와주는 미들웨어
- `body-parser` form 데이터나 AJAX 요청의 데이터를 해석해서 req.body 객체로 만들어 서버가 처리할 수 있게 해주는 미들웨어
- `cookie-parser` 요청에 동봉된 쿠키를 해석해 req.cookies 객체로 만드는 미들웨어

```javascript
const express = require('express')
const morgan = require('morgan')
const helmet = require('helmet')
const cookieParser = require('cookie-parser')

const app = express()

const PORT = 4000;

function handleListening() {
  console.log(`Listening on: http://localhost:${PORT}`);
}

const handleHome = (req, res) => {
  res.send('hello from home');
}

const handleProfile = (req, res) => {
  res.send('You are on my profile')
}

app.use(cookieParser());
app.use(helmet());
app.use(morgan("dev"));

app.get('/', handleHome);

app.get('/profile', handleProfile);

app.listen(PORT, handleListening);

```

