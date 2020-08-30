# Node.js



## I. Node.js + Express.js 웹 서버 설정

### 1. NPM Proect 시작하기

```shell
npm init
```

==> `package.json` 생성



#### Express.js 설치

Node.js 웹 어플리케이션 프레임워크

```shell
npm install express
```

==> `node_modules/`에 `Express.js`를 실행하기 위해 필요한 모듈들이 자동으로 설치



### 2. Express.js 기반 웹 서버 구동

#### `app.js` 생성

```js
// app.js

// node_modules에 있는 express 관련된 모듈들을 불러오기
const express = require('express')
const app = express()

// 비동기로 실행
app.listen(3000, function() {
    console.log("Hello, Express server on port 3000")
})

console.log("end of code")
```



#### 서버 실행

```shell
node app.js
end of code
Hello, Express server on port 3000
```



#### nodemon

`app.js`이 변경될 때마다 이것을 감지하고 자동으로 서버 프로세스를 재시작해주는 모듈

> [참고](https://blog.outsider.ne.kr/649)



```shell
# 내 PC에 전역으로 설치
npm install nodemon -g
```



이제 `nodemon` 명령어로 서버를 시작하면 된다.

```shell
nodemon app.js
```



### URL Routing 처리

```js
...

app.get('/', function(req, res) {
  // res.send("<h1>hi friend!</h1>")
  res.sendFile(__dirname + "/public/main.html")
})
```

`res.send()` 단순한 html 구문을 응답으로 돌려보냄

`res.sendFile()` 해당 경로의 html 파일을 응답으로 보냄

`__dirname`은 현재 폴더의 절대경로를 나타낸다



### Static 디렉터리 설정

static 디렉토리에 있는 파일들은 정적 resource로 사용된다.

```js
...

// Static 디렉토리 설정
app.use(express.static('public'))

...
```



#### 디렉터리 구조

```
📁nodeServerPractice
├─📁node_modules
├─📁public
│ ├─📁images
│ │ └─son.png
│ ├─main.html
│ └─main.js
├─app.js
├─package.json
└─package-lock.json
```



`main.html`

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Document</title>
</head>
<body>
  <h1>main page</h1>
  <p>Run on Node.js server</p>

  <img src="images/son.png" alt="heung-min son">

  <script src="main.js"></script>
</body>
</html>
```



`main.js`

```js
console.log("main.js is loaded")
```



## II. Request, Response 처리 기본

### POST 요청처리

Node.js에서 POST 요청처리를 위해선 `body-parser` 모듈이 필요하다.

> [body-parser를 소개합니다. 하지만, body-parser를 쓰지 마세요.](https://medium.com/@chullino/1%EB%B6%84-%ED%8C%A8%ED%82%A4%EC%A7%80-%EC%86%8C%EA%B0%9C-body-parser%EB%A5%BC-%EC%86%8C%EA%B0%9C%ED%95%A9%EB%8B%88%EB%8B%A4-%ED%95%98%EC%A7%80%EB%A7%8C-body-parser%EB%A5%BC-%EC%93%B0%EC%A7%80-%EB%A7%88%EC%84%B8%EC%9A%94-bc3cbe0b2fd)

> [express 미들웨어 body-parser 모듈](https://velog.io/@yejinh/express-%EB%AF%B8%EB%93%A4%EC%9B%A8%EC%96%B4-bodyParser-%EB%AA%A8%EB%93%88)



```js
// app.js

// Express.js의 Built-in body-parser 사용
app.use(express.json())
app.use(express.urlencoded( {extended : true } ));

...

app.post('/email_post', function(req, res) {
  // get: req.param('email')
  console.log(req.body)
})
```



### View Template Engine을 활용한 응답처리

서버에서 클라이언트에 응답을 줄 때 **데이터를 결합**한 HTML 문서를 전달하기



#### `ejs`라는 Template Engine을 사용

```shell
npm install ejs
```



```js
// app.js
...

app.set('view engine', 'ejs')

...

app.post('/email_post', function(req, res) {
  res.render('email.ejs', {'email': req.body.email})
})
```



`views/email.ejs`

```ejs
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>email ejs form</title>
</head>
<body>
  <p>Your email is <%= email %></p>
</body>
</html>
```





### JSON 활용한 Ajax 처리

Ajax는 새로고침없이 서버에 HTTP 요청을 보내고 응답을 받게 도와줌



#### 1. `form.html`(클라이언트)에서 AJAX 요청

```html
<!DOCTYPE html>
<html>
...

  <button class="ajax-req">Ajax Request</button>

  <p class="result"></p>

  <script>
    document.querySelector('.ajax-req').addEventListener('click', function() {
      const inputData = document.forms[0].elements[0].value
      sendAjax('http://127.0.0.1:3000/ajax_email', inputData)
    })

    function sendAjax(url, data) {
      const data = JSON.stringify({'email': data})
      const xhr = new XMLHttpRequest()
      xhr.open('POST', url)
      xhr.setRequestHeader('Content-Type', "application/json")
      xhr.send(data)

      // 서버에서 받은 응답 처리(예정)
      ...
    }
  </script>
</body>
</html>
```



#### 2. 클라이언트에서 보낸 요청을 Express 서버에서 처리, 응답 송신

```js
// app.js

...

app.post('/ajax_email', function(req, res) {
  // console.log(req.body.email)
  var responseData = {result: 'OK', email: req.body.email}
  res.json(responseData)
})
```



#### 3. 서버에서 받은 응답 처리

```html
  <script>
    document.querySelector('.ajax-req').addEventListener('click', function() {
      var inputData = document.forms[0].elements[0].value
      sendAjax('http://127.0.0.1:3000/ajax_email', inputData)
    })

    function sendAjax(url, data) {
      const data = JSON.stringify({'email': data})
      const xhr = new XMLHttpRequest()
      xhr.open('POST', url)
      xhr.setRequestHeader('Content-Type', "application/json")
      xhr.send(data)

      // 서버에서 받은 응답 처리
	  xhr.addEventListener('load', function() {
      // console.log(xhr.responseText)
      const response = JSON.parse(xhr.responseText)
      if (response.result !== "OK") return;
      document.querySelector('.result').innerHTML = response.email
    })
  </script>
```



## III. Database 연동 기본

### MySQL 연동 설정

우선 MySQL이 설치되어 있어야 함



```mysql
mysql -u root -p

create database jsman;
use jsman;

create table user(
	id INT NOT NULL AUTO_INCREMENT,
    name VARCHAR(20) NOT NULL,
    email VARCHAR(100) NOT NULL,
    password VARCHAR(200) NOT NULL,
    PRIMARY KEY (id)
);

insert into user (id, name, email, password) value (1, 'hong', 'hong@korea.com', 'asdf1234');

...
```



### `mysql` 모듈 설치 및 연동

```shell
npm install mysql
```



[Express.js 공식문서 참고](https://expressjs.com/ko/guide/database-integration.html#mysql)

```js
// app.js

const mysql = require('mysql')

const connection = mysql.createConnection({
    host: 'localhost',
    port: 3306,
    user: 'root',
    password: 'asdf1234',
    database: 'jsman'
})

connection.connect()
```



### MySQL 연동 구현

```js
// app.js

...

app.post('/ajax_email', function(req, res) {
  // console.log(req.body.email)
  const email = req.body.email
  const responseData = {}

  const query = connection.query('select name from user where email="' + email + '"', function(err, rows) {
    if (err) throw err
    if (rows[0]) {
      // console.log(rows[0].name)
      responseData.result = 200
      responseData.name = rows[0].name
    } else {
      // console.log('none: ' + rows[0])
      responseData.result = 400
      responseData.name = ""
    }
    // 비동기로 응답
    res.json(responseData)
  })
})
```



```html
<!-- form.html -->

...

  <script>
    document.querySelector('.ajax-req').addEventListener('click', function() {
      const inputData = document.forms[0].elements[0].value
      sendAjax('http://127.0.0.1:3000/ajax_email', inputData)
    })

    function sendAjax(url, data) {
      const data = JSON.stringify({'email': data})
      const xhr = new XMLHttpRequest()
      xhr.open('POST', url)
      xhr.setRequestHeader('Content-Type', "application/json")
      xhr.send(data)

      xhr.addEventListener('load', function() {
        // console.log(xhr.responseText)
        const response = JSON.parse(xhr.responseText)
        const resultDiv = document.querySelector('.result')
        if (response.result === 200) {
          resultDiv.innerHTML = response.name
        } else {
          resultDiv.innerHTML = "Not Found"
        }
      })
    }
  </script>
</body>
</html>
```



## IV. Router 개선 - 모듈화

### 디렉터리 구조

```
📁nodeServerPractice
├─📁node_modules
├─📁public
│ └─main.html
├─📁router
│ ├─email.js
│ ├─index.js
│ └─main.js
├─app.js
├─package.json
└─package-lock.json
```



### `router/` 폴더로 라우팅 처리할 js 파일 모아놓기 

`router` 폴더 안에 express의 Router를 사용할 `main.js` 생성

```js
// router/main.js
const express = require('express')
const app = express()
const router = express.Router()

router.get('/', function(req, res) {
  res.sendFile(__dirname + '/public/main.html')
})

module.exports = router;  // 다른 파일에서 main.js를 사용할 수 있게 exports 처리
```



`email.js`를 만들어 DB 연동 AJAX 처리 라우팅을 모듈화한다.

```js
// router/email.js
const express = require('express')
const app = express()
const router = express.Router()

// DB Setting
const mysql = require("mysql")

const connection = mysql.createConnection({
  host: 'localhost',
  port: 3306,
  user: 'root',
  password: 'q1w2e3r4',
  database: 'jsman'
})

connection.connect()

// URL Routing
router.post('/form', function(req, res) {
  // get: req.param('email')
  // console.log(req.body)
  res.render('email.ejs', {'email': req.body.email})
})

router.post('/ajax', function(req, res) {
  // console.log(req.body.email)
  const email = req.body.email
  const responseData = {}

  const query = connection.query('select name from user where email="' + email + '"', function(err, rows) {
    if (err) throw err
    if (rows[0]) {
      // console.log(rows[0].name)
      responseData.result = 200
      responseData.name = rows[0].name
    } else {
      // console.log('none: ' + rows[0])
      responseData.result = 400
      responseData.name = ""
    }
    // 비동기로 응답
    res.json(responseData)
  })
})

module.exports = router;
```



이렇게 되면 `app.js`는 가벼워지게 된다.

```js
// app.js

const express = require("express")
const app = express()

// 모듈화한 Routing을 import
const main = require('./router/main')
const email = require('./router/email')

app.listen(3000, function() {
  // console.log("Hello, Express server on port 3000")
})

...

// Router 모듈 사용
app.use('/main', main)
app.use('/email', email)

...

```



`form.html`에서 `form`의 `action` 속성값과 `sendAjax` 함수의 URL을 모듈화한 routing으로 연결

```html
<!-- form.html -->

...

<body>
  <form action="/email/form" method="post">
    <label for="email">Email: </label>
    <input type="text" name="email">
    <input type="submit" value="Submit">
  </form>

  <button class="ajax-req">Ajax Request</button>

  <p class="result"></p>

  <script>
    document.querySelector('.ajax-req').addEventListener('click', function() {
      const inputData = document.forms[0].elements[0].value
      sendAjax('http://127.0.0.1:3000/email/ajax', inputData)
    })
      
      ...
```



`router/index.js`를 만들어 Router 모듈 전체를 관리해보자

```js
// app.js

...

const router = require('./router/index')

...

// Router 모듈화
app.use(router)

```



```js
// index.js

const express = require('express')
const app = express()
const router = express.Router()
const path = require('path')

const main = require('./main')
const email = require('./email')
const join = require('./join')

// URL Routing
router.get('/', function(req, res) {
  res.sendFile(path.join(__dirname, "../public/main.html"))
})

router.use('/main', main)
router.use('/email', email)
router.use('/join', join)

module.exports = router;
```



## V. DB에 데이터 추가

### 디렉터리 구조

```
📁nodeServerPractice
├─📁node_modules
├─📁public
│ ├─join.html	<== 추가
│ └─main.html
├─📁router
│ ├─email.js
│ ├─index.js
│ ├─join.js		<== 추가
│ └─main.js
├─📁views
│ └─main.js
├─app.js
├─package.json
└─package-lock.json
```



`join.js`는 회원가입을 위한 controller이다

```js
const express = require('express')
const app = express()
const router = express.Router()
const path = require('path')

// DB Setting
const mysql = require("mysql")

const connection = mysql.createConnection({
  host: 'localhost',
  port: 3306,
  user: 'root',
  password: 'q1w2e3r4',
  database: 'jsman'
})

connection.connect()

router.get('/', function(req, res) {
  res.sendFile(path.join(__dirname, '../public/join.html'))
})

router.post('/', (req, res) => {
  const body = req.body
  const name = body.nickname
  const email = body.email
  const password = body.password

  const sql = {email: email, name: name, password: password}
  const query = connection.query('insert into user set ?', sql, (err, rows) => {
    if (err) { throw err }
    else res.render('main.ejs', {id: rows.insertId, name: name})
  })
})

module.exports = router;  // 다른 파일에서 main.js를 사용할 수 있게 exports 처리
```



>  `rows`의 형태

```
OkPacket {
    fieldCount: 0,
    affectedRows: 1,
    insertId: 5,
    serverStatus: 2,
    warningCount: 0,
    message: '',
    protocol41: true,
    changedRows: 0
}
```



회원가입에 성공하면 `main.ejs`로 렌더링

```ejs
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Document</title>
</head>
<body>
  <h1>Welcome!</h1>
  <p>This website is running on Node.js server</p>

  <div class="welcome">
    <p><%= name %>! You are <%= id %>th user!</p>
  </div>
  
  <script src="main.js"></script>
</body>
</html>
```



## VI. 패스포트 기반 인증 로직 구현(회원가입, 로그인, 로그아웃)

