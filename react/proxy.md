## Proxy 설정으로 CORS 이슈 해결하기

### `package.json`에 `proxy` 필드 추가

```json
{
    ...
    
 "proxy": "http://localhost:4000",
    
    ...
}
```



### `http-proxy-middleware` 사용

`create-react-app` v2부터 `package.json`에 `proxy` 필드를 지우고 커스터마이징이 가능

- `http-proxy-middleware` 모듈 설치

  ```shell
  npm install http-proxy-middleware --save
  ```

- `axios`나 `fetch` 사용

- [Create React App 공식 문서 참고](https://create-react-app.dev/docs/proxying-api-requests-in-development)

- [http-proxy-middleware npm package 문서 참고](https://www.npmjs.com/package/http-proxy-middleware)

- [Velopert: create-react-app v2 리뷰](https://velog.io/@velopert/create-react-app-v2)

- `src/` 디렉토리에 `setupProxy.js`를 추가한다. 다른 파일에서 import할 필요없다.

  ```js
  // React 서버는 3000번 포트 사용
  
  const { createProxyMiddleware } = require('http-proxy-middleware');
  module.exports = function(app) {
    app.use(
      // createProxyMiddleware('/api', {
      createProxyMiddleware('/', {
        target: 'http://localhost:5000',
        changeOrigin: true,
      })
    );
  };
  ```

  > `/api`로 시작하는 API(`LandingPage.js`에서 `axios.get('/api/hello'))` 등)는 `target`으로 설정된 `http://localhost:5000`로 호출
  >
  > `/api`를 `/`로 바꿀 수 있고, 바꾸면 서버를 재시작해야 한다.
  >
  > 

- `src/components/views/LandingPage/LandingPage.js`

  ```js
  import React, {useEffect} from 'react'
  import axios from 'axios'
  
  function LandingPage() {
    useEffect(() => {
      // axios.get('/api/hello')
      axios.get('/hello')
        .then(res => {
          console.log(res)
        })
    })
    return (
      <div>
        LandingPage
      </div>
    )
  }
  
  export default LandingPage
  ```

- 서버 쪽 `index.js`

  ```js
  ...
  
  app.get('/', (req, res) => {
    res.send('Hello, World!')
  })
  
  app.get('/api/hello', (req, res) => {
    res.send('hello, world!')
  })
  
  ...
  ```

  > 클라이언트 웹 브라우저를 실행하면 `hello, world!`가 아닌 `Hello, World!`가 출력