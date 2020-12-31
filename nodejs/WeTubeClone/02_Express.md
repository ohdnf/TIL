# Express.js



## What is a Server

- 물리적으로 늘 켜져있는 컴퓨터

- 인터넷에 연결되어 요청에 응답하는 컴퓨터



## What is Express.js

> Fast, unopinionated, minimalist web framework for Node.js

서버에서 반복적으로 하는 일을 많은 코드로 미리 만들어 놓은 프레임워크(지름길)



## Installing Express with NPM

### NPM

> Node Package Manager
>
> JavaScript로 만들어진 코드를 공유할 수 있다.

JavaScript를 위한 패키지 관리자. Node.js를 설치하면 자동으로 함께 설치된다.


### NPM으로 Express 설치하기

```shell
$ npm init
```

설정을 완료하면 `package.json` 파일이 생성된다. JSON은 정보를 담는 파일 형식 중 하나이다. 해당 파일이 위치한 경로에서 다음 명령어로 Express를 설치한다. ()`npm` 명령어를 `package.json`이 없는 위치에서 실행하면 또다른 `package.json`을 생성하게 된다.)

```shell
$ npm install express
npm notice created a lockfile as package-lock.json. You should commit this file.
npm WARN wetube@1.0.0 No repository field.

+ express@4.17.1
added 50 packages from 37 contributors and audited 50 packages in 7.854s
found 0 vulnerabilities
```

`package-lock.json` 파일과 `node_modules` 폴더가 함께 설치되었다. express를 사용하기 위한 dependency들이다. Git 등을 활용해 패키지를 공유할 때 dependency들은 공유할 필요가 없다. `package.json` 파일이 있는 경로에서 `npm install` 명령어를 통해 다시 설치가 가능하기 때문이다. 따라서 `.gitignore`와 같은 파일을 만들어 업로드, 버전 추적 등에서 제외시킨다.



## First Express Server

```javascript
const express = require('express')
const app = express()

const PORT = 4000;

function handleListening() {
    console.log(`Listening on: http://localhost:${PORT}`);
}

app.listen(PORT, handleListening);

```

`require`는 매개변수로 받은 문자열과 일치하는 JavaScript 모듈을 찾는다. 먼저 현재 파일이 있는 경로에서 찾고, 없으면 `node_modules` 폴더 안에서 찾는다.

`app.listen(PORT, handleListening)` 함수에 첫 번째 인자로 포트 번호를, 두 번째 인자로 서버를 시작했을 때 실행할 함수를 넘겨줄 수 있다. 현재 서버를 실행하면 콘솔창에 `Listening on: http://localhost:4000`이라는 메시지가 뜨게 된다.

브라우저에서 http://localhost:4000 으로 접속하면 `Cannot GET /` 이라고 뜨는데, 현재 해당 주소를 처리할 라우터가 없기 때문이다.



## Handling Routes with Express

```javascript
const express = require('express')
const app = express()

const PORT = 4000;

function handleListening() {
  console.log(`Listening on: http://localhost:${PORT}`);
}

function handleHome(req, res) {
  // console.log(req);
  res.send('hello from home');
}

function handleProfile(req, res) {
  res.send('You are on my profile')
}

app.get('/', handleHome);

app.get('/profile', handleProfile);

app.listen(PORT, handleListening);

```

`app.get()` 함수를 통해서 라우팅 처리를 할 수 있다.

`/`는 루트 페이지이고 `/profile` 등 URL을 커스터마이징할 수 있고 해당 URL로 요청이 들어왔을 때 응답해줄 콜백 함수를 만들어 두 번째 인자로 넘겨줄 수 있다.

`handleHome(req, res)` 함수에서 `req` 인자는 요청 객체를, `res` 인자는 응답 객체를 나타낸다.



## ES6 on Node.js using Babel

JavaScript 신문법을 쓰고 싶을 때 다시 찾아보자

[Babel 설치](https://babeljs.io/setup#installation)

### nodemon

서버를 종료 후 재시작할 필요없이 코드를 고치고 저장하면 자동으로 서버를 재시작해주는 패키지

```shell
$ npm install nodemon -D
```

`package.json` 파일을 다음과 같이 수정해준다.

```json
{
    ...
    
  "scripts": {
    "start": "nodemon index.js"
  },
  
    ...
    
  "devDependencies": {
    "nodemon": "^2.0.6"
  }
}

```



