# Routing

> 라우팅은 애플리케이션 엔드포인트(URI)의 정의, 그리고 URI가 클라이언트 요청에 응답하는 방식을 말한다.

기본적인 라우트의 예

```javascript
const express = require('express');
const app = express();

// respond with "hello world" when a GET request is made to the homepage
app.get('/', function(req, res) {
  res.send('hello world');
});
```



## 라우트 메소드

> 라우트 메소드는 HTTP 메소드 중 하나로부터 파생되며, `express` 클래스의 인스턴스에 연결된다.

다음 코드는 앱의 루트에 대한 GET 및 POST 메소드에 대해 정의된 라우트의 예다.

```javascript
// GET method route
app.get('/', function (req, res) {
  res.send('GET request to the homepage');
});

// POST method route
app.post('/', function (req, res) {
  res.send('POST request to the homepage');
});
```

Express는 HTTP 메소드에 해당하는 다음과 같은 라우팅 메소드를 지원한다. `get`, `post`, `put`, `head`, `delete`, `options`, `trace`, `copy`, `lock`, `mkcol`, `move`, `purge`, `propfind`, `proppatch`, `unlock`, `report`, `mkactivity`, `checkout`, `merge`, `m-search`, `notify`, `subscribe`, `unsubscribe`, `patch`, `search` 및 `connect`.



특수한 라우팅 메소드인 `app.all()`은 어떠한 HTTP 메소드로부터도 파생되지 않는다. 이 메소드는 모든 요청 메소드에 대해 한 경로에서 미들웨어 함수를 로드하는 데 사용된다.

다음 예에서는, GET, POST, PUT 또는 DELETE 메소드를 사용하는 경우, 또는 [http 모듈](https://nodejs.org/api/http.html#http_http_methods)에서 지원되는 기타 모든 HTTP 요청 메소드를 사용하는 경우 등의 “/secret”에 대한 요청을 위하여 핸들러가 실행된다.

```javascript
app.all('/secret', function (req, res, next) {
  console.log('Accessing the secret section ...');
  next(); // pass control to the next handler
});
```



## 라우트 예시

### `app.js`

```javascript
import express from "express";
import morgan from "morgan";
import helmet from "helmet";
import cookieParser from "cookie-parser";
import { userRouter } from "./router.js";	// const 객체로 반환했기 때문에 destructuring

const app = express();

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

// /user URL로 들어온 모든 요청에 대해 userRouter로 연결
app.use("/user", userRouter);

export default app;
```

`app.use()` 함수는 미들웨어말고도 라우터에도 사용할 수 있다. `app.use("/user", userRouter)`는 브라우저에서 도메인 다음 `/user`가 붙은 요청에 대해 `userRouter`로 처리하겠다는 것을 나타낸다.



### `router.js`

```javascript
import express from "express";

export const userRouter = express.Router();

userRouter.get("/", (req, res) => res.send('User index'));	// 1
userRouter.get("/edit", (req, res) => res.send('User edit'));	// 2
userRouter.get("/password", (req, res) => res.send('User password'));	// 3
```

- 1번은 `GET /user` 요청에 대한 응답을 반환
- 2번은 `GET /user/edit` 요청에 대한 응답을 반환
- 3번은 `GET /user/password` 요청에 대한 응답을 반환



### `init.js`

```javascript
import app from "./app";

const PORT = 4000;

function handleListening() {
  console.log(`Listening on: http://localhost:${PORT}`);
}

app.listen(PORT, handleListening);
```

`package.json` 파일의 `start`를 다음 내용을 수정한다. `"type": "module"`은 모듈 패키지를 `import ... from ...` 구문을 사용해 불러오기 위해 추가한다.

```json
{
  "start": "nodemon init.js",
  "type": "module"
}
```

> #### `"type": "commonjs"` (기본값)
>
> `package.json` 파일에 `type` 속성을 명시하지 않을 경우 기본으로 설정되며, 모듈을 `require()` 구문을 통해 불러와야 한다.
>
> #### `"type": "module"`
>
> `import ... from ...` 구문을 사용해 모듈을 불러와야 한다. 사용자 파일을 읽어오기 위해선 `.js`와 같은 확장자를 명시해야 한다. [참고](https://www.daleseo.com/js-node-es-modules/)