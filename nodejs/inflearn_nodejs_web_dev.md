# Node.js



## I. Node.js + Express.js 웹 서버 설정



### 1. NPM Project 시작하기

```shell
npm init
```

==> `package.json` 생성



#### Express.js 설치

Node.js 웹 어플리케이션 프레임워크

```shell
npm install express
```

==> `node_modules`에 `Express.js`를 실행하기 위해 필요한 모듈들이 자동으로 설치



### 2. Express.js 기반 웹 서버 구동

#### `app.js` 생성

```js
// app.js

// node_modules에 있는 express 관련된 모듈들을 불러오기
var express = require('express')
var app = express()

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

`res.send()`

`res.sendFile()`

`__dirname`은 현재 폴더의 절대경로를 나타낸다



### Static 디렉터리 설정

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
>
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

서버에서 클라이언트에 응답을 줄 때 데이터를 결합한 HTML 문서를 전달하기



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
      var inputData = document.forms[0].elements[0].value
      sendAjax('http://127.0.0.1:3000/ajax_email', inputData)
    })

    function sendAjax(url, data) {
      var data = JSON.stringify({'email': data})
      var xhr = new XMLHttpRequest()
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
      var data = JSON.stringify({'email': data})
      var xhr = new XMLHttpRequest()
      xhr.open('POST', url)
      xhr.setRequestHeader('Content-Type', "application/json")
      xhr.send(data)

      // 서버에서 받은 응답 처리
	  xhr.addEventListener('load', function() {
      // console.log(xhr.responseText)
      var response = JSON.parse(xhr.responseText)
      if (response.result !== "OK") return;
      document.querySelector('.result').innerHTML = response.email
    })
  </script>
```

