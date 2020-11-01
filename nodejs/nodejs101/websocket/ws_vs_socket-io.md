# `ws` 대신 `socket.io`를 쓰는 이유

- ws 프로토콜이 아닌 http 프로토콜 사용
- 네임스페이스, 방 등 특정 소켓과 통신하는 기능 제공
- 

```
views/
│  └─index.html
├─app.js
└─socket.js
```



## `Web Socket`

```js
// app.js
const express = require('express');

const webSocket = require('./socket');

const app = express();
app.set('port', process.env.PORT || 8005);

const server = app.listen(app.get('port'), () => {
  console.log(app.get('port'), '번 포트에서 대기중');
});

webSocket(server, app, sessionMiddleware);
```



```html
<!-- views/index.html -->
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>GIF 채팅방</title>
</head>
<body>
  <div>F12를 눌러 개발자 모드에서 console 탭과 network 탭을 확인하세요</div>
  <h1>Web Socket</h1>
  <script>
    const webSocket = new WebSocket('ws://localhost:8005');
    webSocket.onopen = function () {
      console.log('서버와 웹 소켓 연결 성공!');
    });
    webSocket.onmessage = function (event) {
      console.log(event.data);
      webSocket.send('클라이언트에서 서버로 답장을 보냅니다');
    };
  </script>
</body>
</html>
```

## `Socket.IO`

```js
// app.js
const WebSocket = require('ws');

module.exports = (server) => {
  const wss = new WebSocket.Server({ server });

  wss.on('connection', (ws, req) => { // 웹 소켓 연결 시
    // 클라이언트의 IP를 알아내는 방법
    // req.headers['x-forwarded-for'] ==> proxy 서버의 ip
    const ip = req.headers['x-forwarded-for'] || req.connection.remoteAddress;
    console.log('새로운 클라이언트 접속', ip);
    ws.on('message', (message) => { // 클라이언트로부터 메시지 수신 시
      console.log(message);
    });
    ws.on('error', (error) => { // 에러 시
      console.error(error);
    });
    ws.on('close', () => {  // 브라우저 종료 등으로 연결 종료 시
      console.log('클라이언트 접속 해제', ip);
      clearInterval(ws.interval); // 메모리 누수 방지
    });

    ws.interval = setInterval(() => { // 3초마다 클라이언트로 메시지 전송
      if (ws.readyState === ws.OPEN) {  // 연결 상태 검사
        ws.send('서버에서 클라이언트로 메시지를 보냅니다.')
      }
    }, 3000);
  });
};
```



## `Socket.IO`와 비교

```js
// app.js
const SocketIO = require('socket.io');

module.exports = (server) => {
  const io = SocketIO(server, { path: '/socket.io' });

  io.on('connection', (socket) => { // 웹소켓 연결 시
    const req = socket.request; // 요청 객체
    // socket.request.res ==> 응답 객체
    const ip = req.headers['x-forwarded-for'] || req.connection.remoteAddress;
    console.log('새로운 클라이언트 접속!', ip, socket.id, req.ip);
    socket.on('disconnect', () => { // 연결 종료 시
      console.log('클라이언트 접속 해제', ip, socket.id);
      clearInterval(socket.interval);
    });
    socket.on('error', (error) => { // 에러 시
      console.error(error);
    });
    // 커스텀 이벤트
    socket.on('reply', (data) => { // 클라이언트로부터 메시지
      console.log(data);
    });
    socket.interval = setInterval(() => { // 3초마다 클라이언트로 메시지 전송
      // socket.emit(이벤트 이름, 데이터);
      socket.emit('news', 'Hello Socket.IO');
    }, 3000);
  });
};
```



```html
<!-- views/index.html -->
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>GIF 채팅방</title>
</head>
<body>
  <div>F12를 눌러 개발자 모드에서 console 탭과 network 탭을 확인하세요</div>
  <h1>Web Socket</h1>
  <script src="/socket.io/socket.io.js"></script>
  <script>
    const socket = io.connect('http://localhost:8005', {
      path: '/socket.io',
       // Socket.IO가 폴링 방식으로 연결 후(웹 소켓을 지원하지 않는 브라우저를 위해)
       // 웹 소켓으로 업그레이드하는 방식을 처음부터 웹 소켓만 사용하도록 설정
      transports: ['websocket'],
    });
    socket.on('news', function (data) {
      console.log(data);
      socket.emit('reply', 'Hello Node.JS');
    });
  </script>
</body>
</html>
```